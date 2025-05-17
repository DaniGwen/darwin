/*
 * main.cpp
 *
 * Created on: 2011. 1. 4.
 * Author: robotis
 * Modified for Edge TPU Object Detection via Unix Domain Socket
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstdio>
#include <sstream>

// Headers for Unix Domain Sockets
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h> // For errno and strerror

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

// --- Socket Configuration ---
// Define the path for the Unix Domain Socket
const char *SOCKET_PATH = "/tmp/darwin_detector.sock";
// Buffer size for receiving data from Python (adjust as needed)
const int RECV_BUFFER_SIZE = 4096; // Note: This buffer size is less critical now that we read size first

#define INI_FILE_PATH       "config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"

void change_current_dir()
{
    char exepath[1024] = {0};
    if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

// Structure to hold parsed detection data received from Python
struct ParsedDetection
{
    std::string label;
    float score;
    float xmin, ymin, xmax, ymax; // Normalized coordinates [0.0, 1.0]
};

// Function to parse the detection string received from Python
// Expected format: "label score xmin ymin xmax ymax\nlabel score xmin ymin xmax ymax\n..."
std::vector<ParsedDetection> parse_detection_output(const std::string &output)
{
    std::vector<ParsedDetection> detections;
    std::stringstream ss(output);
    std::string line;

    while (std::getline(ss, line, '\n'))
    {
        std::stringstream line_ss(line);
        ParsedDetection det;
        std::string label_str; // Read label as a string

        // Use string stream to parse label and then numerical values
        if (line_ss >> label_str >> det.score >> det.xmin >> det.ymin >> det.xmax >> det.ymax)
        {
            det.label = label_str;
            detections.push_back(det);
        }
    }
    return detections;
}


// Basic function to draw bounding boxes on the RGB image
void DrawBoundingBox(Image *image, const ParsedDetection &detection)
{
    if (!image || !image->m_ImageData) return;

    int img_width = image->m_Width;
    int img_height = image->m_Height;

    // Bounding box coordinates are normalized [0.0, 1.0]
    int xmin = static_cast<int>(detection.xmin * img_width);
    int ymin = static_cast<int>(detection.ymin * img_height);
    int xmax = static_cast<int>(detection.xmax * img_width);
    int ymax = static_cast<int>(detection.ymax * img_height);

    // Clamp coordinates to image boundaries
    xmin = std::max(0, std::min(xmin, img_width - 1));
    ymin = std::max(0, std::min(ymin, img_height - 1));
    xmax = std::max(0, std::min(xmax, img_width - 1));
    ymax = std::max(0, std::min(ymax, img_height - 1));

    // Draw a simple red box (assuming RGB_PIXEL_SIZE = 3)
    unsigned char r = 255, g = 0, b = 0;

    // Draw lines (simplified)
    for (int x = xmin; x <= xmax; ++x) {
        if (ymin >= 0 && ymin < img_height) {
            unsigned char *p = &image->m_ImageData[(ymin * img_width + x) * image->m_PixelSize];
            p[0] = r; p[1] = g; p[2] = b;
        }
        if (ymax >= 0 && ymax < img_height) {
            unsigned char *p = &image->m_ImageData[(ymax * img_width + x) * image->m_PixelSize];
            p[0] = r; p[1] = g; p[2] = b;
        }
    }
    for (int y = ymin; y <= ymax; ++y) {
        if (xmin >= 0 && xmin < img_width) {
            unsigned char *p = &image->m_ImageData[(y * img_width + xmin) * image->m_PixelSize];
            p[0] = r; p[1] = g; p[2] = b;
        }
        if (xmax >= 0 && xmax < img_width) {
            unsigned char *p = &image->m_ImageData[(y * img_width + xmax) * image->m_PixelSize];
            p[0] = r; p[1] = g; p[2] = b;
        }
    }

    // For simplicity, printing to console:
    std::cout << "Detected: " << detection.label << " (" << detection.score << ") at ["
              << xmin << "," << ymin << "]-[" << xmax << "," << ymax << "]" << std::endl;

    // Drawing text label is more complex and omitted here.
}


int main(void)
{
    printf("\n===== Head tracking with Object Detection via Unix Domain Socket =====\n\n");

    change_current_dir();

    minIni *ini = new minIni(INI_FILE_PATH);

    // --- Setup Unix Domain Socket Server ---
    int server_sock, client_sock;
    struct sockaddr_un server_addr;

    // Remove existing socket file if it exists
    unlink(SOCKET_PATH);

    // Create socket
    server_sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_sock < 0)
    {
        std::cerr << "ERROR: Failed to create socket: " << strerror(errno) << std::endl;
        return -1;
    }

    // Bind socket to address
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sun_family = AF_UNIX;
    strncpy(server_addr.sun_path, SOCKET_PATH, sizeof(server_addr.sun_path) - 1);

    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        std::cerr << "ERROR: Failed to bind socket to " << SOCKET_PATH << ": " << strerror(errno) << std::endl;
        close(server_sock);
        return -1;
    }

    // Listen for connections
    if (listen(server_sock, 5) < 0) // Allow up to 5 pending connections
    {
        std::cerr << "ERROR: Failed to listen on socket: " << strerror(errno) << std::endl;
        close(server_sock);
        return -1;
    }

    std::cout << "INFO: Waiting for Python detector script connection on " << SOCKET_PATH << "..." << std::endl;

    // Accept a connection (this will block until the Python script connects)
    client_sock = accept(server_sock, NULL, NULL);
    if (client_sock < 0)
    {
        std::cerr << "ERROR: Failed to accept client connection: " << strerror(errno) << std::endl;
        close(server_sock);
        return -1;
    }

    std::cout << "INFO: Python detector script connected." << std::endl;
    // Close the listening socket, we only expect one client (the detector script)
    close(server_sock);


    // Image buffer for the output frame with detections drawn on it
    Image *rgb_display_frame = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);
    // Ensure camera is providing RGB data (Image::RGB_PIXEL_SIZE = 3)

    mjpg_streamer *streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    // --- Head Tracking Setup ---
    // ... (Your existing Head Tracking setup code) ...
    //////////////////// Framework Initialize ////////////////////////////
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);
    if (MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        printf("Fail to initialize Motion Manager!\n");
        // Cleanup socket before exiting
        close(client_sock);
        unlink(SOCKET_PATH);
        return 0;
    }
    MotionManager::GetInstance()->LoadINISettings(ini);
    MotionManager::GetInstance()->AddModule((MotionModule *)Head::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
    MotionManager::GetInstance()->SetEnable(true);
    /////////////////////////////////////////////////////////////////////

    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 8);
    Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 8);

    std::cout << "INFO: Starting main loop..." << std::endl;
    while (1)
    {
        LinuxCamera::GetInstance()->CaptureFrame();
        Image *current_cam_rgb_frame = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;

        if (!current_cam_rgb_frame || !current_cam_rgb_frame->m_ImageData)
        {
            usleep(10000); // Wait if frame not ready
            continue;
        }

        // --- Send Frame Data to Python Script ---
        int frame_width = current_cam_rgb_frame->m_Width;
        int frame_height = current_cam_rgb_frame->m_Height;
        size_t frame_data_size = frame_width * frame_height * current_cam_rgb_frame->m_PixelSize;

        // Send width and height first (as 4-byte integers)
        if (send(client_sock, &frame_width, sizeof(frame_width), 0) < 0 ||
            send(client_sock, &frame_height, sizeof(frame_height), 0) < 0)
        {
            std::cerr << "ERROR: Failed to send frame dimensions: " << strerror(errno) << std::endl;
            break; // Exit loop on send error
        }

        // Send the raw image data
        if (send(client_sock, current_cam_rgb_frame->m_ImageData, frame_data_size, 0) < 0)
        {
            std::cerr << "ERROR: Failed to send frame data: " << strerror(errno) << std::endl;
            break; // Exit loop on send error
        }

        // --- Receive Detection Results from Python Script ---
        // First, receive the size of the detection string
        uint32_t result_size = 0;
        ssize_t bytes_received = recv(client_sock, &result_size, sizeof(result_size), 0);
        if (bytes_received <= 0) // 0 indicates connection closed, <0 indicates error
        {
            if (bytes_received == 0)
            {
                std::cerr << "ERROR: Python script closed the connection." << std::endl;
            }
            else
            {
                std::cerr << "ERROR: Failed to receive result size: " << strerror(errno) << std::endl;
            }
            break; // Exit loop on receive error or connection closed
        }

        // Receive the actual detection string
        std::string detection_output(result_size, '\0'); // Pre-allocate string buffer
        size_t total_received = 0;
        while (total_received < result_size)
        {
            ssize_t current_recv = recv(client_sock, &detection_output[total_received], result_size - total_received, 0);
            if (current_recv <= 0)
            {
                if (current_recv == 0)
                {
                    std::cerr << "ERROR: Python script closed connection while receiving data." << std::endl;
                }
                else
                {
                    std::cerr << "ERROR: Failed to receive detection data: " << strerror(errno) << std::endl;
                }
                break; // Exit inner loop on error
            }
            total_received += current_recv;
        }

        if (total_received < result_size)
        {
            // Partial data received, indicates an issue
            break; // Exit main loop
        }

        // --- Parse and Use Detection Results ---
        std::vector<ParsedDetection> detections = parse_detection_output(detection_output);

        bool target_found = false;
        Point2D tracked_object_center_for_head;

        // Example: Find the first "person" detection
        for (const auto &det : detections)
        {
            DrawBoundingBox(rgb_display_frame, det); // Draw all detections

            // --- Modified: Check if the detected label is "person" ---
            if (det.label == "person") // Look for the label "person"
            {
                // Calculate center of the bounding box in original image pixel coordinates
                tracked_object_center_for_head.X = (det.xmin + det.xmax) / 2.0 * Camera::WIDTH;
                tracked_object_center_for_head.Y = (det.ymin + det.ymax) / 2.0 * Camera::HEIGHT;
                target_found = true;
                // For simplicity, track the first person found.
                // For better tracking, you might want to track the largest or closest person.
                break; // Stop searching after finding the first person
            }
        }

        // --- Head Tracking ---
        if (target_found)
        {
            Point2D P_err;
            // Normalize pixel coordinates to -1.0 to 1.0 range (approx)
            P_err.X = (tracked_object_center_for_head.X - (Camera::WIDTH / 2.0)) / (Camera::WIDTH / 2.0);
            P_err.Y = (tracked_object_center_for_head.Y - (Camera::HEIGHT / 2.0)) / (Camera::HEIGHT / 2.0);

            Head::GetInstance()->MoveTracking(P_err);
        }
        else
        {
            // No target found, perhaps make the head look forward or scan
            // Head::GetInstance()->MoveTracking(Point2D(0.0, 0.0)); // Stop active tracking
        }

        streamer->send_image(rgb_display_frame);

        // usleep(10000); // Optional delay
    }

    // --- Cleanup ---
    std::cout << "INFO: Exiting main loop. Cleaning up..." << std::endl;
    close(client_sock); // Close the client connection socket
    unlink(SOCKET_PATH); // Remove the socket file

    delete rgb_display_frame;
    delete ini;
    delete streamer;

    return 0;
}
