#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <signal.h>
#include <memory>

// ONNX Runtime & Darwin Framework headers...
#include <onnxruntime_cxx_api.h>
#include "LinuxDARwIn.h"

// --- Constants ---
const std::string BALANCE_MODEL_PATH = "balance_actor.onnx";
const std::string GRASP_MODEL_PATH = "grasp_actor.onnx"; // Vision-based model

const int NUM_MOTORS_TOTAL = 20;
const int NUM_SENSORS = 24;
const int CAMERA_WIDTH = 160;
const int CAMERA_HEIGHT = 120;
const int CAMERA_CHANNELS = 4;
const int IMAGE_SIZE = CAMERA_WIDTH * CAMERA_HEIGHT * CAMERA_CHANNELS;

#pragma pack(push, 1)
struct VisionSensorData { /* as defined in supervisor */ };
struct MotorCommands { /* as defined in supervisor */ };
#pragma pack(pop)

// Global flag, signal handler, etc...
bool g_is_running = true;
void signal_handler(int signum) { g_is_running = false; }

// --- Main Program ---
int main() {
    signal(SIGINT, signal_handler);
    
    // --- Setup TCP Connection to Vision Supervisor ---
    // (This code would connect to your C++ supervisor)
    
    // --- Load BOTH Models ---
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "hybrid-agent");
    Ort::SessionOptions session_options;
    
    auto balance_session = std::make_unique<Ort::Session>(env, BALANCE_MODEL_PATH.c_str(), session_options);
    auto grasp_session = std::make_unique<Ort::Session>(env, GRASP_MODEL_PATH.c_str(), session_options);
    
    std::vector<float> final_motor_commands(NUM_MOTORS_TOTAL, 0.0f);

    while (g_is_running) {
        // 1. Receive the combined data packet from the supervisor
        // VisionSensorData received_data;
        // recv(socket, &received_data, ...);

        // 2. Prepare inputs for both models
        // std::vector<float> sensor_observation(received_data.sensor_values, ...);
        // std::vector<float> image_observation_float = preprocess_image(received_data.image_data);

        // 3. Run the Balance Model
        // It outputs commands for 18 joints
        // std::vector<float> balance_action = get_action_from_model(*balance_session, sensor_observation);
        
        // 4. Run the Grasping Model
        // It outputs commands for the 3 right arm joints
        // std::vector<float> grasp_action = get_action_from_model(*grasp_session, image_observation_float);

        // 5. --- MERGE ACTIONS ---
        // Copy all body commands from the balance model
        // for (int i = 0; i < 18; ++i) {
        //     final_motor_commands[i] = balance_action[i];
        // }
        
        // Overwrite the right arm commands with the output from the vision model
        // final_motor_commands[Robot::JointData::ID_R_SHOULDER_PITCH - 1] = grasp_action[0];
        // final_motor_commands[Robot::JointData::ID_R_SHOULDER_ROLL - 1]  = grasp_action[1];
        // final_motor_commands[Robot::JointData::ID_R_ELBOW - 1]           = grasp_action[2];

        // 6. Send the final, merged command vector to the robot
        // apply_action(final_motor_commands, cm730);

        usleep(16000);
    }

    // Cleanup...
    return 0;
}
// Other helper functions (initialize_robot, apply_action, etc.) would need to be included
