#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <signal.h>
#include <memory> // For std::unique_ptr

// ONNX Runtime C++ API
#include <onnxruntime_cxx_api.h>

// Darwin-OP Framework Headers
#include "LinuxDARwIn.h"

// --- State Machine Definition ---
enum RobotState {
    BALANCING,
    WALKING,
    FALLEN_FORWARD,
    FALLEN_BACKWARD,
    GETTING_UP
};

// --- Constants ---
// --- MODIFICATION: Define paths for all three models ---
const std::string BALANCE_MODEL_PATH = "balance_actor.onnx";
const std::string WALK_MODEL_PATH = "walk_actor.onnx";
const std::string STANDUP_MODEL_PATH = "standup_actor.onnx"; // Model for getting up

const int NUM_MOTORS_CONTROLLED = 18;
const int OBSERVATION_DIM = 24;
const int ACTION_DIM = 18;
const int ID_WRIST = 21;
const int ID_GRIPPER = 22;

// Thresholds for state transitions
const float FALL_THRESHOLD_PITCH = 1.0; // Radians (~57 degrees)
const float RECOVERED_THRESHOLD_PITCH = 0.35; // Radians (~20 degrees)

// Global flag to handle graceful shutdown on Ctrl+C
bool g_is_running = true;

// --- Function Prototypes ---
void signal_handler(int signum);
bool initialize_robot(Robot::CM730 &cm730);
std::vector<float> read_observation(Robot::CM730 &cm730);
void apply_action(const std::vector<float>& action_rad, Robot::CM730 &cm730);
std::vector<float> get_action_from_model(Ort::Session& session, const std::vector<float>& observation);


// --- Main Program ---
int main() {
    signal(SIGINT, signal_handler);
    std::cout << "Starting Darwin-OP Multi-Model RL Agent..." << std::endl;

    Robot::LinuxCM730 linux_cm730("/dev/ttyUSB0");
    Robot::CM730 cm730(&linux_cm730);
    if (!initialize_robot(cm730)) {
        return -1;
    }

    // --- MODIFICATION: Load all three models ---
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "darwin-agent");
    Ort::SessionOptions session_options;

    std::cout << "Loading models..." << std::endl;
    auto balance_session = std::make_unique<Ort::Session>(env, BALANCE_MODEL_PATH.c_str(), session_options);
    auto walk_session = std::make_unique<Ort::Session>(env, WALK_MODEL_PATH.c_str(), session_options);
    auto standup_session = std::make_unique<Ort::Session>(env, STANDUP_MODEL_PATH.c_str(), session_options);
    std::cout << "All models loaded successfully." << std::endl;

    // --- FSM Initialization ---
    RobotState current_state = BALANCING;
    auto last_state_change_time = std::chrono::steady_clock::now();

    sleep(2);

    std::cout << "Starting main control loop..." << std::endl;
    while (g_is_running) {
        std::vector<float> observation = read_observation(cm730);
        std::vector<float> action;

        float current_pitch = observation[19]; // Pitch is a key indicator for state

        // --- State Machine Logic ---
        switch (current_state) {
            case BALANCING:
                std::cout << "\rState: BALANCING  " << std::flush;
                action = get_action_from_model(*balance_session, observation);
                
                // Transition to WALK after 3 seconds of stable balancing
                if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - last_state_change_time).count() > 3) {
                    current_state = WALKING;
                    last_state_change_time = std::chrono::steady_clock::now();
                    std::cout << "\nTransitioning to WALKING state." << std::endl;
                }
                break;

            case WALKING:
                std::cout << "\rState: WALKING    " << std::flush;
                action = get_action_from_model(*walk_session, observation);
                break;

            case GETTING_UP:
                std::cout << "\rState: GETTING UP " << std::flush;
                action = get_action_from_model(*standup_session, observation);

                // Transition to BALANCING once the robot is upright
                if (std::abs(current_pitch) < RECOVERED_THRESHOLD_PITCH) {
                    current_state = BALANCING;
                    last_state_change_time = std::chrono::steady_clock::now();
                    std::cout << "\nTransitioning to BALANCING state." << std::endl;
                }
                break;
            
            // Note: Fallen states don't produce actions, they just transition to GETTING_UP
            case FALLEN_FORWARD:
            case FALLEN_BACKWARD:
                current_state = GETTING_UP;
                last_state_change_time = std::chrono::steady_clock::now();
                std::cout << "\nTransitioning to GETTING_UP state." << std::endl;
                // Use a default "tucked in" action while waiting for the get_up model to take over
                action = std::vector<float>(ACTION_DIM, 0.0f); 
                break;
        }

        // --- Universal Fall Detection ---
        if (current_state != GETTING_UP) {
            if (current_pitch > FALL_THRESHOLD_PITCH) {
                current_state = FALLEN_FORWARD;
                std::cout << "\nDetected a forward fall!" << std::endl;
            } else if (current_pitch < -FALL_THRESHOLD_PITCH) {
                current_state = FALLEN_BACKWARD;
                std::cout << "\nDetected a backward fall!" << std::endl;
            }
        }
        
        apply_action(action, cm730);
        usleep(16000);
    }

    std::cout << "\nShutdown signal received. Turning off all motor torque." << std::endl;
    for (int i = 1; i <= 22; ++i) { // Turn off all 22 motors
        cm730.WriteWord(i, Robot::MX28::P_TORQUE_ENABLE, 0, 0);
    }

    return 0;
}

// --- Helper function to run inference on a given model ---
std::vector<float> get_action_from_model(Ort::Session& session, const std::vector<float>& observation) {
    Ort::AllocatorWithDefaultOptions allocator;
    const char* input_name = session.GetInputName(0, allocator);
    const char* output_name = session.GetOutputName(0, allocator);
    std::vector<int64_t> input_dims = {1, OBSERVATION_DIM};
    
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, observation.data(), observation.size(), input_dims.data(), input_dims.size());
    
    std::vector<const char*> input_names = {input_name};
    std::vector<const char*> output_names = {output_name};

    auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_names.data(), &input_tensor, 1, output_names.data(), 1);

    float* output_data = output_tensors[0].GetTensorMutableData<float>();
    size_t output_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();

    return std::vector<float>(output_data, output_data + output_size);
}


// --- Function Implementations (mostly unchanged) ---
void signal_handler(int signum) {
    g_is_running = false;
}

bool initialize_robot(Robot::CM730 &cm730) {
    if (cm730.Connect() == false) {
        std::cerr << "Failed to connect to CM730." << std::endl;
        return false;
    }

    std::cout << "Initializing main body motors..." << std::endl;
    for (int id = 1; id <= NUM_MOTORS_CONTROLLED; ++id) {
        cm730.WriteWord(id, Robot::MX28::P_TORQUE_ENABLE, 1, 0);
        cm730.WriteByte(id, Robot::MX28::P_P_GAIN, 32, 0);
    }
    
    std::cout << "Locking extra hand servos in neutral position..." << std::endl;
    const int NEUTRAL_POSITION = 2048;
    cm730.WriteWord(Robot::JointData::ID_R_WRIST, Robot::MX28::P_TORQUE_ENABLE, 1, 0);
    cm730.WriteWord(Robot::JointData::ID_R_WRIST, Robot::MX28::P_GOAL_POSITION_L, NEUTRAL_POSITION, 0);
    cm730.WriteWord(Robot::JointData::ID_R_GRIPPER, Robot::MX28::P_TORQUE_ENABLE, 1, 0);
    cm730.WriteWord(Robot::JointData::ID_R_GRIPPER, Robot::MX28::P_GOAL_POSITION_L, NEUTRAL_POSITION, 0);
    
    std::cout << "Robot initialized." << std::endl;
    return true;
}

std::vector<float> read_observation(Robot::CM730 &cm730) {
    std::vector<float> observation(OBSERVATION_DIM);

    int present_position_val;
    for (int id = 1; id <= NUM_MOTORS_CONTROLLED; ++id) {
        if (cm730.ReadWord(id, Robot::MX28::P_PRESENT_POSITION_L, &present_position_val, 0) == Robot::CM730::SUCCESS) {
            observation[id - 1] = Robot::Angle::Value2Radian(present_position_val);
        } else {
            observation[id - 1] = 0.0f;
        }
    }

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
    
    observation[21] = 0.0f; // X
    observation[22] = 0.0f; // Y
    observation[23] = 0.0f; // Z
    
    return observation;
}

void apply_action(const std::vector<float>& action_rad, Robot::CM730 &cm730) {
    if (action_rad.size() != NUM_MOTORS_CONTROLLED) return;

    for (int i = 0; i < NUM_MOTORS_CONTROLLED; ++i) {
        int motor_id = i + 1;
        int goal_position_val = Robot::Angle::Radian2Value(action_rad[i]);
        cm730.WriteWord(motor_id, Robot::MX28::P_GOAL_POSITION_L, goal_position_val, 0);
    }
}