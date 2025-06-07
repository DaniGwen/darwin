*
 * Description: A cross-platform C++ TCP server designed to run within the
 * Darwin-OP framework. This is the final, robust version with a self-contained
 * clamp function for maximum compatibility with different C++ compilers.
 */

#include <iostream>
#include <vector>
#include <string>
#include <cmath>     // For sin() in placeholder logic
#include <chrono>    // For timing
#include <algorithm> // Needed for std::min and std::max

// --- Platform-Specific Networking Includes ---
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

// --- IMPORTANT DATA STRUCTURES ---
#pragma pack(push, 1)
struct SensorData
{
    double joint_positions[20];
    double roll, pitch, yaw;
};
struct MotorCommands
{
    double joint_targets[20];
};
#pragma pack(pop)

// --- HELPER FUNCTION ---
// A simple clamp function to ensure compatibility with older C++ standards
// that don't have std::clamp (pre-C++17).
template <typename T>
const T &clamp(const T &value, const T &low, const T &high)
{
    return std::min(high, std::max(value, low));
}

// --- Main Logic Processing ---
void process_logic(const SensorData &sensors, MotorCommands &commands, double time)
{
    const std::vector<std::pair<double, double>> joint_limits = {
        {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}, {-2.25, 2.25}};

    for (int i = 0; i < 20; ++i)
    {
        commands.joint_targets[i] = 0.0;
    }

    double amplitude = 0.5;
    double frequency = 0.5;

    commands.joint_targets[0] = -amplitude * sin(2 * M_PI * frequency * time);
    commands.joint_targets[1] = amplitude * sin(2 * M_PI * frequency * time);
    commands.joint_targets[8] = -amplitude * 0.5 * sin(2 * M_PI * frequency * time + M_PI);
    commands.joint_targets[9] = amplitude * 0.5 * sin(2 * M_PI * frequency * time + M_PI);

    // Final Safety Check: Clamp all values using our helper function
    for (int i = 0; i < 20; ++i)
    {
        double min_pos = joint_limits[i].first;
        double max_pos = joint_limits[i].second;
        // Notice we call clamp(...) instead of std::clamp(...)
        commands.joint_targets[i] = clamp(commands.joint_targets[i], min_pos, max_pos);
    }
}

// --- Main Server Function ---
int main()
{
    const int PORT = 1234;
    int server_fd, client_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::cerr << "WSAStartup failed" << std::endl;
        return 1;
    }
#endif

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        std::cerr << "Socket creation failed" << std::endl;
        return 1;
    }
    std::cout << "Server socket created successfully." << std::endl;

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        std::cerr << "Bind failed" << std::endl;
        return 1;
    }
    std::cout << "Socket bound to port " << PORT << "." << std::endl;

    if (listen(server_fd, 3) < 0)
    {
        std::cerr << "Listen failed" << std::endl;
        return 1;
    }
    std::cout << "Server listening... Waiting for a Webots client to connect." << std::endl;

    if ((client_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
    {
        std::cerr << "Accept failed" << std::endl;
        return 1;
    }
    std::cout << "Webots client connected!" << std::endl;

    SensorData received_sensors;
    MotorCommands commands_to_send;
    auto start_time = std::chrono::high_resolution_clock::now();

    while (true)
    {
        int bytes_received = recv(client_socket, (char *)&received_sensors, sizeof(SensorData), 0);

        if (bytes_received <= 0)
        {
            std::cout << "Client disconnected or connection error." << std::endl;
            break;
        }

        if (bytes_received != sizeof(SensorData))
        {
            std::cerr << "Warning: Received incomplete sensor packet." << std::endl;
            continue;
        }

        auto current_time = std::chrono::high_resolution_clock::now();
        double elapsed_time = std::chrono::duration<double>(current_time - start_time).count();
        process_logic(received_sensors, commands_to_send, elapsed_time);

        if (send(client_socket, (const char *)&commands_to_send, sizeof(MotorCommands), 0) == -1)
        {
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