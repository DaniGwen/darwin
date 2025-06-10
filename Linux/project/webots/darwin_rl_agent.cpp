#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <signal.h>

// ONNX Runtime C++ API
#include <onnxruntime_cxx_api.h>

// Darwin-OP Framework Headers
#include "LinuxDARwIn.h"

// --- Constants ---
const std::string ONNX_MODEL_PATH = "darwin_actor.onnx";
const int NUM_MOTORS_CONTROLLED = 18; // The model only controls 18 motors
const int OBSERVATION_DIM = 24;
const int ACTION_DIM = 18;

// Global flag to handle graceful shutdown on Ctrl+C
bool g_is_running = true;

// --- Function Prototypes ---
void signal_handler(int signum);
bool initialize_robot(Robot::CM730 &cm730);
std::vector<float> read_observation(Robot::CM730 &cm730);
void apply_action(const std::vector<float>& action_rad, Robot::CM730 &cm730);

// --- Main Program ---
int main() {
    signal(SIGINT, signal_handler);
    std::cout << "Starting Darwin-OP RL Agent..." << std::endl;

    Robot::LinuxCM730 linux_cm730("/dev/ttyUSB0");
    Robot::CM730 cm730(&linux_cm730);
    if (!initialize_robot(cm730)) {
        std::cerr << "Failed to initialize the robot. Exiting." << std::endl;
        return -1;
    }

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "darwin-agent");
    Ort::SessionOptions session_options;
    Ort::Session session(env, ONNX_MODEL_PATH.c_str(), session_options);

    Ort::AllocatorWithDefaultOptions allocator;
    const char* input_name = session.GetInputName(0, allocator);
    const char* output_name = session.GetOutputName(0, allocator);
    std::vector<int64_t> input_dims = {1, OBSERVATION_DIM};
    
    std::cout << "ONNX model loaded successfully." << std::endl;
    sleep(2);

    std::cout << "Starting main control loop..." << std::endl;
    while (g_is_running) {
        std::vector<float> observation = read_observation(cm730);
        
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, observation.data(), observation.size(), input_dims.data(), input_dims.size());
        
        std::vector<const char*> input_names = {input_name};
        std::vector<const char*> output_names = {output_name};

        auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_names.data(), &input_tensor, 1, output_names.data(), 1);

        float* output_data = output_tensors[0].GetTensorMutableData<float>();
        size_t output_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();

        std::vector<float> action(output_data, output_data + output_size);
        
        apply_action(action, cm730);
        
        usleep(16000); // ~60Hz control loop
    }

    std::cout << "\nShutdown signal received. Turning off all motor torque." << std::endl;
    // Turn off torque for ALL motors, including the hand
    for (int i = 1; i <= 20; ++i) {
        cm730.WriteWord(i, Robot::MX28::P_TORQUE_ENABLE, 0, 0);
    }

    return 0;
}

// --- Function Implementations ---
void signal_handler(int signum) {
    g_is_running = false;
}

bool initialize_robot(Robot::CM730 &cm730) {
    if (cm730.Connect() == false) {
        std::cerr << "Failed to connect to CM730." << std::endl;
        return false;
    }

    // --- MODIFICATION: Initialize and lock extra hand servos ---
    std::cout << "Initializing main body motors..." << std::endl;
    for (int id = 1; id <= NUM_MOTORS_CONTROLLED; ++id) {
        cm730.WriteWord(id, Robot::MX28::P_TORQUE_ENABLE, 1, 0);
        cm730.WriteByte(id, Robot::MX28::P_P_GAIN, 32, 0);
    }
    
    std::cout << "Locking extra hand servos in neutral position..." << std::endl;
    const int NEUTRAL_POSITION = 2048; // Center position for most Dynamixels
    // Wrist (ID 21)
    cm730.WriteWord(Robot::JointData::ID_R_WRIST, Robot::MX28::P_TORQUE_ENABLE, 1, 0);
    cm730.WriteWord(Robot::JointData::ID_R_WRIST, Robot::MX28::P_GOAL_POSITION_L, 2084, 0);
    // Gripper (ID 22)
    cm730.WriteWord(Robot::JointData::ID_R_GRIPPER, Robot::MX28::P_TORQUE_ENABLE, 1, 0);
    cm730.WriteWord(Robot::JointData::ID_R_GRIPPER, Robot::MX28::P_GOAL_POSITION_L, NEUTRAL_POSITION, 0);
    // --- END MODIFICATION ---

    std::cout << "Robot initialized." << std::endl;
    return true;
}

std::vector<float> read_observation(Robot::CM730 &cm730) {
    std::vector<float> observation(OBSERVATION_DIM);

    // Read only the 18 joint positions the model was trained on
    int present_position_val;
    for (int id = 1; id <= NUM_MOTORS_CONTROLLED; ++id) {
        if (cm730.ReadWord(id, Robot::MX28::P_PRESENT_POSITION_L, &present_position_val, 0) == Robot::CM730::SUCCESS) {
            observation[id - 1] = Robot::Angle::Value2Radian(present_position_val);
        } else {
            observation[id - 1] = 0.0f;
        }
    }

    // Read IMU data (Roll, Pitch, Yaw)
    int imu_roll, imu_pitch, imu_yaw;
    if (cm730.ReadWord(Robot::CM730::ID_CM, Robot::CM730::P_GYRO_Y_L, &imu_roll, 0) == Robot::CM730::SUCCESS &&
        cm730.ReadWord(Robot::CM730::ID_CM, Robot::CM730::P_GYRO_X_L, &imu_pitch, 0) == Robot::CM730::SUCCESS &&
        cm730.ReadWord(Robot::CM730::ID_CM, Robot::CM730::P_GYRO_Z_L, &imu_yaw, 0) == Robot::CM730::SUCCESS) {
        
        observation[18] = (imu_roll - 512) * (M_PI / 180.0);
        observation[19] = (imu_pitch - 512) * (M_PI / 180.0);
        observation[20] = (imu_yaw - 512) * (M_PI / 180.0);
    } else {
        observation[18] = 0.0f; observation[19] = 0.0f; observation[20] = 0.0f;
    }
    
    // Set simulated position to zero, as the real robot can't know this.
    observation[21] = 0.0f; // X
    observation[22] = 0.0f; // Y
    observation[23] = 0.0f; // Z
    
    return observation;
}

void apply_action(const std::vector<float>& action_rad, Robot::CM730 &cm730) {
    if (action_rad.size() != NUM_MOTORS_CONTROLLED) return;

    // Send commands only to the 18 motors the model controls
    for (int i = 0; i < NUM_MOTORS_CONTROLLED; ++i) {
        int motor_id = i + 1;
        int goal_position_val = Robot::Angle::Radian2Value(action_rad[i]);
        cm730.WriteWord(motor_id, Robot::MX28::P_GOAL_POSITION_L, goal_position_val, 0);
    }
}