/*
 * main.cpp
 *
 * Created on: 2011. 1. 4.
 * Author: robotis
 * Modified for Edge TPU Object Detection via Unix Domain Socket
 * Removed repeated motion framework initialization from main loop.
 * Added explicit error scaling and deadband for centering.
 * Added automatic startup of the Python detector script.
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
#include <sstream> // Required for std::stringstream
#include <cmath>   // Required for std::abs
#include <cstdlib> // Required for system()

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

// --- Python Script Configuration ---
// IMPORTANT: Set the correct path to your Python detector script
const char *PYTHON_SCRIPT_PATH = "/home/darwin/darwin/aiy-maker-kit/examples/custom_detect_objects.py"; // Corrected path

#define INI_FILE_PATH       "config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0" // Verify this path is correct!

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

// Initializes the Unix Domain Socket server and waits for connection
// Returns client_sock file descriptor on success, -1 on failure
int initialize_socket_server()
{
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

    return client_sock;
}

// Initializes the Camera
// Returns true on success, false on failure
bool initialize_camera(minIni* ini)
{
    if (!ini) {
        std::cerr << "ERROR: INI file not provided for camera initialization." << std::endl;
        return false;
    }
    std::cout << "INFO: Initializing camera..." << std::endl;
    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);
    std::cout << "INFO: Camera initialized." << std::endl;
    return true;
}

// Initializes the MJPG Streamer
// Returns streamer pointer on success, nullptr on failure
mjpg_streamer* initialize_streamer()
{
    std::cout << "INFO: Initializing MJPG streamer..." << std::endl;
    mjpg_streamer *streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);
    if (!streamer) {
        std::cerr << "ERROR: Failed to create MJPG streamer." << std::endl;
        return nullptr;
    }
    std::cout << "INFO: MJPG streamer initialized." << std::endl;
    return streamer;
}


// Handles sending frame data over the socket
// Returns true on success, false on failure
bool send_frame_data(int client_sock, Image* frame)
{
    if (!frame || !frame->m_ImageData) {
        return false;
    }

    int frame_width = frame->m_Width;
    int frame_height = frame->m_Height;
    size_t frame_data_size = frame_width * frame_height * frame->m_PixelSize;

    // Send width and height first (as 4-byte integers)
    if (send(client_sock, &frame_width, sizeof(frame_width), 0) < 0 ||
        send(client_sock, &frame_height, sizeof(frame_height), 0) < 0)
    {
        std::cerr << "ERROR: Failed to send frame dimensions: " << strerror(errno) << std::endl;
        return false;
    }

    // Send the raw image data
    if (send(client_sock, frame->m_ImageData, frame_data_size, 0) < 0)
    {
        std::cerr << "ERROR: Failed to send frame data: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

// Handles receiving detection results over the socket and parsing them
// Returns vector of detections on success, empty vector on failure or no detections
std::vector<ParsedDetection> receive_detection_results(int client_sock)
{
    std::string detection_output;
    uint32_t result_size = 0;

    // Receive the size of the detection string
    ssize_t bytes_received = recv(client_sock, &result_size, sizeof(result_size), 0);
    if (bytes_received <= 0)
    {
        if (bytes_received == 0)
        {
            std::cerr << "ERROR: Python script closed the connection." << std::endl;
        }
        else
        {
            std::cerr << "ERROR: Failed to receive result size: " << strerror(errno) << std::endl;
        }
        return {}; // Return empty vector on error or closed connection
    }

    // Receive the actual detection string
    detection_output.resize(result_size, '\0'); // Resize string buffer
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
            return {}; // Return empty vector on error
        }
        total_received += current_recv;
    }

    // Parse and return detections
    return parse_detection_output(detection_output);
}

// Handles the main processing loop (capture, send, receive, track)
// Returns 0 on successful exit, -1 on error
int run_main_loop(int client_sock, mjpg_streamer* streamer, minIni* ini)
{
    if (!ini) {
        // Error message already printed in main, just return false
        return false;
    }

    // Initialize Motion Framework once before the loop
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);

    if (MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        std::cerr << "ERROR: Failed to initialize Motion Manager before loop." << std::endl;
        return false;
    }

    MotionManager::GetInstance()->LoadINISettings(ini);
    MotionManager::GetInstance()->AddModule((MotionModule *)Head::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
    MotionManager::GetInstance()->SetEnable(true);

    // Explicitly enable head joints and set gains once before the loop
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);

    // --- Set P-Gains for Head Tracking ---
    // These values control how strongly the motor reacts to the error signal.
    // Tune these alongside the error scaling factors below for best results.
    Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 8); // Set P-gain for pan
    Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 8); // Set P-gain for tilt
    // You might need to increase these values (e.g., to 12, 16, or higher)
    // if the head isn't moving quickly enough. Be cautious of oscillation.

    // --- Tuning Parameters for Centering ---
    // These values control how the calculated error is applied to the head movement.
    const double PAN_ERROR_SCALE = 0.9; // Scale factor for horizontal error (tune this: 0.1 to 2.0 usually)
    const double TILT_ERROR_SCALE = 0.9; // Scale factor for vertical error (tune this: 0.1 to 2.0 usually)
    const double PAN_DEADBAND_DEG = 0.5; // Deadband in degrees for pan (tune this: 0.5 to 3.0 usually)
    const double TILT_DEADBAND_DEG = 0.5; // Deadband in degrees for tilt (tune this: 0.5 to 3.0 usually)


    // Image buffer for the output frame with detections drawn on it
    Image *rgb_display_frame = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    if (!rgb_display_frame) {
        std::cerr << "ERROR: Failed to create display frame buffer." << std::endl;
        return -1;
    }

    // Tracking State Variables
    int NoTargetCount = 0;
    const int NoTargetMaxCount = 30; // Number of frames to wait before initiating scan (tune this)

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
        if (!send_frame_data(client_sock, current_cam_rgb_frame))
        {
            // send_frame_data prints error message
            break; // Exit loop on send error
        }

        // --- Receive and Parse Detection Results ---
        std::vector<ParsedDetection> detections = receive_detection_results(client_sock);

        // Check if receive_detection_results indicated a connection error by returning empty
        // If it returned empty due to a connection error, receive_detection_results
        // would have printed an error, and the next send/receive will likely fail,
        // leading to a break.


        // Copy original camera frame to the display frame for drawing
        memcpy(rgb_display_frame->m_ImageData, current_cam_rgb_frame->m_ImageData,
               current_cam_rgb_frame->m_NumberOfPixels * current_cam_rgb_frame->m_PixelSize);

        // --- Process Detections and Update Tracking State ---
        bool person_found_in_frame = false; // Flag for the current frame
        Point2D tracked_object_center_for_head;

        // Example: Find the first "person" detection
        for (const auto &det : detections)
        {
            DrawBoundingBox(rgb_display_frame, det); // Draw all detections

            // --- Check if the detected label is "person" ---
            if (det.label == "person") // Look for the label "person"
            {
                // Calculate center of the bounding box in original image pixel coordinates
                tracked_object_center_for_head.X = (det.xmin + det.xmax) / 2.0 * Camera::WIDTH;
                tracked_object_center_for_head.Y = (det.ymin + det.ymax) / 2.0 * Camera::HEIGHT;
                person_found_in_frame = true; // A person was found in this frame
                // For simplicity, track the first person found.
                // For better tracking, you might want to track the largest or closest person.
                break; // Stop searching after finding the first person
            }
        }

        // --- Head Tracking State Management ---
        if (person_found_in_frame)
        {
            NoTargetCount = 0; // Reset the counter since a target was found
            Point2D P_err;

            // --- Calculate angular error similar to original BallTracker ---
            // Calculate pixel offset from center
            Point2D pixel_offset_from_center;
            pixel_offset_from_center.X = tracked_object_center_for_head.X - (Camera::WIDTH / 2.0);
            pixel_offset_from_center.Y = tracked_object_center_for_head.Y - (Camera::HEIGHT / 2.0);

            // --- Invert X-axis error to correct tracking direction ---
            pixel_offset_from_center.X *= -1;
            // Invert Y-axis (if needed, depends on your framework's coordinate system)
            pixel_offset_from_center.Y *= -1;

            // Scale pixel offset to angles (degrees)
            P_err.X = pixel_offset_from_center.X * (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH);
            P_err.Y = pixel_offset_from_center.Y * (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT);

            // --- Apply Deadband and Error Scaling for Centering ---
            // Apply deadband: only move if error is greater than the threshold
            if (std::abs(P_err.X) < PAN_DEADBAND_DEG) {
                P_err.X = 0.0; // Set error to zero within the deadband
            } else {
                // Apply additional scaling outside the deadband
                P_err.X *= PAN_ERROR_SCALE;
            }

            if (std::abs(P_err.Y) < TILT_DEADBAND_DEG) {
                P_err.Y = 0.0; // Set error to zero within the deadband
            } else {
                // Apply additional scaling outside the deadband
                P_err.Y *= TILT_ERROR_SCALE;
            }

            // Pass angular error to MoveTracking
            Head::GetInstance()->MoveTracking(P_err); // Actively track the person
        }
        else // No person found in the current frame
        {
            if(NoTargetCount < NoTargetMaxCount)
            {
                // Continue tracking based on the last known position or stop active tracking
                Head::GetInstance()->MoveTracking(); // Original BallTracker behavior (might hold last pos)
                // Head::GetInstance()->MoveTracking(Point2D(0.0, 0.0)); // Alternative: stop active tracking and center head slowly
                NoTargetCount++; // Increment counter
            }
            else
            {
                // No target for too long, initiate scan or return to initial position
                Head::GetInstance()->InitTracking(); // Return to initial tracking position/scan
                // NoTargetCount remains at or above NoTargetMaxCount
            }
        }

        streamer->send_image(rgb_display_frame);

        // usleep(10000); // Optional delay
    }

    // Cleanup
    delete rgb_display_frame;
    return 0; // Indicate successful loop termination (though unlikely in this infinite loop)
}

// Handles cleanup of resources
void cleanup(int client_sock, minIni* ini, mjpg_streamer* streamer)
{
    std::cout << "INFO: Cleaning up..." << std::endl;
    if (client_sock >= 0) {
        close(client_sock); // Close the client connection socket
    }
    // The server socket was closed after accepting the connection.
    // Remove the socket file
    unlink(SOCKET_PATH);

    // MotionManager, Head, LinuxMotionTimer are singletons managed by the framework.
    // Depending on the framework's design, they might have their own cleanup methods
    // or are expected to persist for the application's lifetime.
    // Explicit deletion of singletons is often discouraged unless the framework
    // provides specific cleanup functions.

    // Delete dynamically allocated objects
    if (ini) delete ini;
    if (streamer) delete streamer;
    // rgb_display_frame is deleted in run_main_loop before returning
}


int main(void)
{
    printf("\n===== Head tracking with Object Detection via Unix Domain Socket =====\n\n");

    change_current_dir();

    minIni *ini = new minIni(INI_FILE_PATH);
    if (!ini) {
        std::cerr << "ERROR: Failed to load INI file." << std::endl;
        return -1;
    }

    // --- Auto-start the Python detector script ---
    // Construct the command to execute the Python script
    std::string command = "python3 ";
    command += PYTHON_SCRIPT_PATH;
    // Add '&' to run the command in the background, so the C++ program doesn't wait for it to finish
    command += " &";
    std::cout << "INFO: Starting Python detector script: " << command << std::endl;
    int system_return = system(command.c_str());

    if (system_return != 0) {
        std::cerr << "WARNING: Failed to start Python script using system(). Make sure the path is correct and python3 is in PATH." << std::endl;
        // Note: system() return value can vary; 0 usually means success, but check man page for specifics.
    }
    // Give the Python script a moment to start and create the socket
    usleep(1000000); // 1 second delay (adjust if needed)


    // --- Initialize Socket Server ---
    int client_sock = initialize_socket_server();
    if (client_sock < 0) {
        delete ini; // Clean up ini
        return -1;
    }

    // --- Initialize Camera ---
    if (!initialize_camera(ini)) {
        std::cerr << "ERROR: Failed to initialize camera." << std::endl;
        cleanup(client_sock, ini, nullptr); // Pass nullptr for streamer as it's not initialized yet
        return -1;
    }

    // --- Initialize MJPG Streamer ---
    mjpg_streamer *streamer = initialize_streamer();
    if (!streamer) {
        std::cerr << "ERROR: Failed to initialize MJPG streamer." << std::endl;
        cleanup(client_sock, ini, nullptr); // Streamer is null
        return -1;
    }


    // --- Run Main Processing Loop ---
    // Pass ini to the main loop so it can be used for re-initialization
    int loop_status = run_main_loop(client_sock, streamer, ini);

    // --- Cleanup Resources ---
    cleanup(client_sock, ini, streamer);

    return loop_status;
}
