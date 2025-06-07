/*
 * Description: A C++ TCP server that loads a pre-trained ONNX neural network
 * model to control the Darwin-OP robot in Webots. This version contains the
 * corrected header path for onnxruntime and compatibility fixes for v1.13.1.
 */

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <memory>

// --- Corrected ONNX Runtime C++ API Include Path ---
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

// Platform-Specific Networking Includes
#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
#endif

// --- DATA STRUCTURES ---
#pragma pack(push, 1)
struct SensorData {
    double joint_positions[20];
    double roll, pitch, yaw;
};
struct MotorCommands {
    double joint_targets[20];
};
#pragma pack(pop)

// --- HELPER FUNCTION for clamping values ---
template<typename T>
const T& clamp(const T& value, const T& low, const T& high) {
    return std::min(high, std::max(value, low));
}

// --- GLOBAL VARIABLES for ONNX Runtime ---
struct ONNX_Model {
    Ort::Env env;
    Ort::Session session;
    Ort::AllocatorWithDefaultOptions allocator;

    // These vectors hold the raw C-style strings for the API call.
    std::vector<const char*> input_node_names;
    std::vector<const char*> output_node_names;

    // These vectors own the memory for the names and will clean it up automatically.
    std::vector<Ort::AllocatedStringPtr> input_name_holder;
    std::vector<Ort::AllocatedStringPtr> output_name_holder;


    ONNX_Model(const char* model_path) : 
        env(ORT_LOGGING_LEVEL_WARNING, "NN_INFERENCE"),
        session(env, model_path, Ort::SessionOptions{nullptr}) {
        
        // --- Corrected method for getting model input/output names for ORT v1.13 ---
        // Get the number of inputs and outputs
        size_t num_input_nodes = session.GetInputCount();
        size_t num_output_nodes = session.GetOutputCount();

        // Get the input node names
        for (size_t i = 0; i < num_input_nodes; i++) {
            Ort::AllocatedStringPtr name = session.GetInputNameAllocated(i, allocator);
            input_node_names.push_back(name.get());
            input_name_holder.push_back(std::move(name));
        }

        // Get the output node names
        for (size_t i = 0; i < num_output_nodes; i++) {
            Ort::AllocatedStringPtr name = session.GetOutputNameAllocated(i, allocator);
            output_node_names.push_back(name.get());
            output_name_holder.push_back(std::move(name));
        }
    }
};

// --- Main Logic Processing ---
void process_logic(const SensorData& sensors, MotorCommands& commands, ONNX_Model& model) {
    // --- 1. Prepare Input Tensor ---
    const int input_size = 23;
    std::vector<float> input_tensor_values(input_size);
    for(int i = 0; i < 20; ++i) input_tensor_values[i] = (float)sensors.joint_positions[i];
    input_tensor_values[20] = (float)sensors.roll;
    input_tensor_values[21] = (float)sensors.pitch;
    input_tensor_values[22] = (float)sensors.yaw;

    std::vector<int64_t> input_tensor_shape = {1, input_size};
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_tensor_values.data(), input_tensor_values.size(), input_tensor_shape.data(), input_tensor_shape.size());

    // --- 2. Run Inference ---
    auto output_tensors = model.session.Run(Ort::RunOptions{nullptr}, model.input_node_names.data(), &input_tensor, 1, model.output_node_names.data(), 1);

    // --- 3. Extract Output and Clamp ---
    float* output_values = output_tensors[0].GetTensorMutableData<float>();
    const int output_size = 20;

    std::vector<std::pair<double, double>> joint_limits;
    joint_limits.reserve(20);
    for (int i = 0; i < 20; ++i) {
        joint_limits.push_back({-2.25, 2.25});
    }

    for (int i = 0; i < output_size; ++i) {
        commands.joint_targets[i] = clamp((double)output_values[i], joint_limits[i].first, joint_limits[i].second);
    }
}

// --- Main Server Function ---
int main() {
    // --- Load the Neural Network Model ---
    std::unique_ptr<ONNX_Model> model;
    try {
        model = std::make_unique<ONNX_Model>("./darwin_model.onnx");
        std::cout << "Neural network model loaded successfully." << std::endl;
    } catch (const Ort::Exception& e) {
        std::cerr << "ERROR loading ONNX model: " << e.what() << std::endl;
        return 1;
    }
    
    const int PORT = 1234;
    int server_fd, client_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed" << std::endl;
        return 1;
    }
#endif

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        std::cerr << "Socket creation failed" << std::endl;
        return 1;
    }
    
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        return 1;
    }
    
    if (listen(server_fd, 3) < 0) {
        std::cerr << "Listen failed" << std::endl;
        return 1;
    }
    std::cout << "Server listening... Waiting for a Webots client to connect." << std::endl;

    if ((client_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
        std::cerr << "Accept failed" << std::endl;
        return 1;
    }
    std::cout << "Webots client connected!" << std::endl;

    SensorData received_sensors;
    MotorCommands commands_to_send;
    
    while (true) {
        int bytes_received = recv(client_socket, (char*)&received_sensors, sizeof(SensorData), 0);
        if (bytes_received <= 0) {
            std::cout << "Client disconnected or connection error." << std::endl;
            break;
        }
        
        process_logic(received_sensors, commands_to_send, *model);

        if (send(client_socket, (const char*)&commands_to_send, sizeof(MotorCommands), 0) == -1) {
            std::cerr << "Failed to send motor commands." << std::endl;
            break;
        }
    }

    std::cout << "Closing sockets." << std::endl;
#ifdef _WIN32
    closesocket(client_socket);
    closesocket(server_fd);
    WSACleanup();
#else
    close(client_socket);
    close(server_fd);
#endif

    return 0;
}