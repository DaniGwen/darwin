/*
 * =====================================================================================
 *
 * Filename:  darwin_rl_agent.cpp
 *
 * Description:  C++ application to run a trained ONNX reinforcement learning
 * model on a real Darwin-OP robot. It handles hardware init,
 * sensor reading, odometry estimation, and model inference.
 *
 * Author:  Your Name
 *
 * =====================================================================================
 */

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <unistd.h>
#include <signal.h>

// --- ONNX Runtime ---
// The main C++ API for running ONNX models.
#include <onnxruntime_cxx_api.h>

// --- Darwin-OP Framework ---
// These are the standard headers for controlling the Darwin-OP hardware.
// The exact paths might differ slightly in your specific framework version.
#include "LinuxDARwIn.h"

// --- Constants ---
const std::string ONNX_MODEL_PATH = "darwin_actor.onnx";
const int NUM_MOTORS = 18; // Must match your training setup (excludes head/neck)
const int OBSERVATION_DIM = 24; // 18 motors + 3 IMU + 3 odometry
const int ACTION_DIM = 18;

// Global flag to handle graceful shutdown on Ctrl+C
bool g_is_running = true;

// --- Function Prototypes ---
void signal_handler(int signum);
bool initialize_robot(Robot::CM730 &cm730);
std::vector<float> read_observation(Robot::CM730 &cm730, Robot::LinuxCM730 &linux_cm730, float &current_x, float &current_y, float &current_yaw);
void apply_action(const std::vector<float>& action_rad, Robot::CM730 &cm730);

// --- Main Program ---
int main() {
    // Register the signal handler for Ctrl+C
    signal(SIGINT, signal_handler);

    std::cout << "Starting Darwin-OP RL Agent..." << std::endl;

    // 1. Initialize Robot Hardware
    Robot::LinuxCM730 linux_cm730("/dev/ttyUSB0"); // Path to your USB2Dynamixel
    Robot::CM730 cm730(&linux_cm730);
    if (!initialize_robot(cm730)) {
        std::cerr << "Failed to initialize the robot. Exiting." << std.endl;
        return -1;
    }

    // 2. Initialize ONNX Runtime
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "darwin-agent");
    Ort::SessionOptions session_options;
    Ort::Session session(env, ONNX_MODEL_PATH.c_str(), session_options);

    Ort::AllocatorWithDefaultOptions allocator;
    const char* input_name = session.GetInputName(0, allocator);
    const char* output_name = session.GetOutputName(0, allocator);
    std::vector<int64_t> input_dims = {1, OBSERVATION_DIM};
    
    std::cout << "ONNX model loaded successfully." << std::endl;
    std::cout << "Input Name: " << input_name << ", Output Name: " << output_name << std::endl;

    // 3. Initialize Odometry Variables
    float odometry_x = 0.0f;
    float odometry_y = 0.0f;
    float odometry_yaw = 0.0f;
    
    // Give the robot a moment to settle after initialization
    sleep(2);

    // 4. Main Inference Loop
    std::cout << "Starting main control loop..." << std::endl;
    while (g_is_running) {
        // Get the current state of the robot
        std::vector<float> observation = read_observation(cm730, linux_cm730, odometry_x, odometry_y, odometry_yaw);
        
        // Create input tensor for the ONNX model
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, observation.data(), observation.size(), input_dims.data(), input_dims.size());
        
        std::vector<const char*> input_names = {input_name};
        std::vector<const char*> output_names = {output_name};

        // Run model inference
        auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_names.data(), &input_tensor, 1, output_names.data(), 1);

        // Get the action from the output tensor
        float* output_data = output_tensors[0].GetTensorMutableData<float>();
        std::vector<float> action(output_data, output_data + ACTION_DIM);
        
        // Send the action to the robot's motors
        apply_action(action, cm730);

        // Maintain a consistent control frequency (e.g., ~60Hz)
        usleep(16000); // 16ms
    }

    // Graceful shutdown
    std::cout << "\nShutdown signal received. Turning off motor torque." << std::endl;
    for (int i = 1; i <= NUM_MOTORS; ++i) {
        cm730.WriteWord(i, Robot::MX28::P_TORQUE_ENABLE, 0, 0);
    }

    return 0;
}


// --- Function Implementations ---

// Handles Ctrl+C interruption to exit the main loop cleanly.
void signal_handler(int signum) {
    g_is_running = false;
}

// Initializes the robot's motors.
bool initialize_robot(Robot::CM730 &cm730) {
    if (cm730.Connect() == false) {
        std::cerr << "Failed to connect to CM730." << std::endl;
        return false;
    }

    // Set Torque Enable and P-Gains for all 18 motors
    for (int id = 1; id <= NUM_MOTORS; ++id) {
        cm730.WriteWord(id, Robot::MX28::P_TORQUE_ENABLE, 1, 0); // Enable torque
        cm730.WriteByte(id, Robot::MX28::P_P_GAIN, 32, 0); // Set P-Gain to a default value
    }
    
    // Set robot to a standing/ready pose (all joints at 0 degrees)
    std::vector<float> ready_pose(NUM_MOTORS, 0.0f);
    apply_action(ready_pose, cm730);

    std::cout << "Robot initialized and torque enabled." << std::endl;
    return true;
}

// Reads all sensors and estimates position to create the 24-element observation vector.
std::vector<float> read_observation(Robot::CM730 &cm730, Robot::LinuxCM730 &linux_cm730, float &current_x, float &current_y, float &current_yaw) {
    std::vector<float> observation(OBSERVATION_DIM);

    // 1. Read Joint Positions (Motors 1-18)
    int present_position_val;
    for (int id = 1; id <= NUM_MOTORS; ++id) {
        if (cm730.ReadWord(id, Robot::MX28::P_PRESENT_POSITION_L, &present_position_val, 0) == Robot::CM730::SUCCESS) {
            // Convert from MX-28 raw value (0-4095) to radians (-pi to pi)
            observation[id - 1] = Robot::Angle::Value2Radian(present_position_val);
        } else {
            observation[id - 1] = 0.0f; // Default to 0 on read failure
        }
    }

    // 2. Read IMU Data (Roll, Pitch, Yaw)
    int imu_roll, imu_pitch, imu_yaw;
    if (cm730.ReadWord(Robot::CM730::ID_CM, Robot::CM730::P_GYRO_Y_L, &imu_roll, 0) == Robot::CM730::SUCCESS &&
        cm730.ReadWord(Robot::CM730::ID_CM, Robot::CM730::P_GYRO_X_L, &imu_pitch, 0) == Robot::CM730::SUCCESS &&
        cm730.ReadWord(Robot::CM730::ID_CM, Robot::CM730::P_GYRO_Z_L, &imu_yaw, 0) == Robot::CM730::SUCCESS) {
        
        // Convert from raw sensor value to radians
        observation[18] = (imu_roll - 512) * (M_PI / 180.0);
        observation[19] = (imu_pitch - 512) * (M_PI / 180.0);
        observation[20] = (imu_yaw - 512) * (M_PI / 180.0);
    } else {
        observation[18] = 0.0f;
        observation[19] = 0.0f;
        observation[20] = 0.0f;
    }
    
    // --- 3. Simple Odometry for Position Estimation ---
    // This is a very basic estimation. A more advanced robot would use a Kalman filter.
    // We'll estimate forward movement based on the robot's pitch.
    float pitch = observation[19];
    float step_length = 0.0;
    
    // A simple heuristic: if the robot is pitched forward, it's likely moving forward.
    // This needs heavy tuning based on your robot's actual gait.
    if (pitch > 0.1) { // 0.1 radians is about 5.7 degrees
        step_length = pitch * 0.01; // This coefficient (0.01) needs tuning
    }

    // Update position based on current yaw
    current_x += step_length * cos(current_yaw);
    current_y += step_length * sin(current_yaw);
    current_yaw = observation[20]; // Update yaw directly from IMU

    observation[21] = current_x;
    observation[22] = current_y;
    // The Z position is harder to estimate. We can approximate it based on leg joint angles
    // or keep it simple for now. Let's use a rough approximation based on ankle pitch.
    float left_ankle_pitch = observation[13]; // Joint 14
    float right_ankle_pitch = observation[12]; // Joint 13
    observation[23] = -0.26 + 0.1 * (cos(left_ankle_pitch) + cos(right_ankle_pitch)); // Heuristic for Z height


    return observation;
}

// Sends the action (18 joint targets in radians) to the robot's motors.
void apply_action(const std::vector<float>& action_rad, Robot::CM730 &cm730) {
    if (action_rad.size() != NUM_MOTORS) return;

    for (int i = 0; i < NUM_MOTORS; ++i) {
        int motor_id = i + 1;
        // Convert radians back to MX-28 raw value (0-4095)
        int goal_position_val = Robot::Angle::Radian2Value(action_rad[i]);
        cm730.WriteWord(motor_id, Robot::MX28::P_GOAL_POSITION_L, goal_position_val, 0);
    }
}