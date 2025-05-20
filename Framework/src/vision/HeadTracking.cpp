/*
 * HeadTracking.cpp
 *
 * Created on: May 17, 2025
 * Author: Your Name
 * Description: Implementation of the HeadTracking singleton class.
 *              Receives initialized Motion Framework (MotionManager, Head, CM730) pointers.
 *              Added explicit includes for stringstream fix.
 *              Defined SOCKET_PATH here.
 *              Added detailed logging to Run() loop, including pre-capture log and initial delay.
 *              Added Python script startup to Initialize().
 *              Added CM730 pointer and LED color control logic.
 *              Added debug logging for LED commands and a basic LED test.
 *              Added member variable and getter to expose the detected label.
 *              Added member variable and getter to expose the tracked object's center coordinates.
 */

#include "HeadTracking.h"
#include <string>   // Explicitly include string
#include <vector>   // Explicitly include vector
#include <sstream>  // Include sstream where stringstream is used
#include <cmath>    // For std::abs
#include <cstring>  // For memcpy
#include <cstdio>   // For printf (used in DrawBoundingBox)
#include <unistd.h> // For usleep
#include <cstdlib>  // For system()
#include <mutex>    // For std::mutex (optional, but good practice for shared data)

// Define socket path here (only once)
const char *SOCKET_PATH = "/tmp/darwin_detector.sock";

// --- Python Script Configuration ---
// IMPORTANT: Set the correct path to your Python detector script
const char *PYTHON_SCRIPT_PATH = "/home/darwin/darwin/aiy-maker-kit/examples/custom_detect_objects.py";

// Static member initialization (singleton instance)
HeadTracking *HeadTracking::GetInstance()
{
    static HeadTracking instance; // Guaranteed to be created once upon first call
    return &instance;
}

// Private constructor
HeadTracking::HeadTracking()
    : client_socket_(-1),
      streamer_(nullptr),
      ini_settings_(nullptr),
      cm730_(nullptr),
      rgb_display_frame_(nullptr),
      no_target_count_(0),
      pan_error_scale_(0.5), // Default values (can be overridden by INI)
      tilt_error_scale_(0.5),
      pan_deadband_deg_(1.0),
      tilt_deadband_deg_(1.0),
      black_color_(0),
      current_detected_label_("none"),         // Initialize detected label
      current_tracked_object_center_(0.0, 0.0) // Initialize tracked object center (X, Y)
{
    // Constructor is intentionally minimal.
    // Initialization that might fail or requires external resources
    // should be done in the Initialize() method.
}

// Destructor
HeadTracking::~HeadTracking()
{
    Cleanup();
    // Note: ini_settings_ is owned by main and should not be deleted here.
}

bool HeadTracking::Initialize(minIni *ini, CM730 *cm730)
{
    ini_settings_ = ini;
    cm730_ = cm730;   

    // Basic check if passed pointers are valid
    if (!cm730_)
    {
        std::cerr << "ERROR: HeadTracking initialization failed: Invalid MotionManager, Head, or CM730 pointer passed." << std::endl;
        // No cleanup needed for motion components as they are not owned.
        return false;
    }

    // --- Initialize Components ---

    // 0. Auto-start the Python detector script (moved from main)
    // Construct the command to execute the Python script
    std::string command = "python3 ";
    command += PYTHON_SCRIPT_PATH;
    // Add '&' to run the command in the background, so the C++ program doesn't wait for it to finish
    command += " &";
    std::cout << "INFO: Starting Python detector script: " << command << std::endl;
    int system_return = system(command.c_str());

    if (system_return != 0)
    {
        std::cerr << "WARNING: Failed to start Python script using system(). Make sure the path is correct and python3 is in PATH." << std::endl;
        // Note: system() return value can vary; 0 usually means success, but check man page for specifics.
    }
    // Give the Python script a moment to start and create the socket
    usleep(2000000); // 2 second delay (adjust if needed)

    // 1. Initialize Socket Server and wait for Python connection
    client_socket_ = InitializeSocketServer();
    if (client_socket_ < 0)
    {
        std::cerr << "ERROR: HeadTracking initialization failed: Socket server setup failed." << std::endl;
        return false;
    }
    
    // 3. Initialize MJPG Streamer
    if (!InitializeStreamer())
    {
        std::cerr << "ERROR: HeadTracking initialization failed: Streamer setup failed." << std::endl;
        Cleanup(); // Clean up already initialized resources
        return false;
    }

    // 5. Create display frame buffer
    rgb_display_frame_ = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    if (!rgb_display_frame_)
    {
        std::cerr << "ERROR: HeadTracking initialization failed: Failed to create display frame buffer." << std::endl;
        Cleanup(); // Clean up already initialized resources
        return false;
    }

    // 6. Load tuning parameters from INI if available
    if (ini_settings_)
    {
        pan_error_scale_ = ini_settings_->getd("HeadTracking", "PanErrorScale", pan_error_scale_);
        tilt_error_scale_ = ini_settings_->getd("HeadTracking", "TiltErrorScale", tilt_error_scale_);
        pan_deadband_deg_ = ini_settings_->getd("HeadTracking", "PanDeadbandDeg", pan_deadband_deg_);
        tilt_deadband_deg_ = ini_settings_->getd("HeadTracking", "TiltDeadbandDeg", tilt_deadband_deg_);
        std::cout << "INFO: Loaded HeadTracking tuning parameters from INI." << std::endl;
    }

    std::cout << "INFO: HeadTracking initialization successful." << std::endl;
    return true;
}

void HeadTracking::Run()
{
    // Check if essential components are initialized before running
    if (client_socket_ < 0 || !streamer_ || !rgb_display_frame_ || !ini_settings_)
    {
        std::cerr << "ERROR: HeadTracking not fully initialized. Cannot run." << std::endl;
        return;
    }

    std::cout << "INFO: Starting HeadTracking main loop..." << std::endl;

    // Add a small delay before the first frame capture (kept for robustness)
    usleep(500000); // 0.5 second delay

    while (1)
    {
        // --- Capture Frame ---
        LinuxCamera::GetInstance()->CaptureFrame();
        // std::cout << "DEBUG: CaptureFrame() returned." << std::endl; // Added log - can be noisy

        Image *current_cam_rgb_frame = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;
        // std::cout << "DEBUG: Frame captured." << std::endl; // Added log - can be noisy

        if (!current_cam_rgb_frame || !current_cam_rgb_frame->m_ImageData)
        {
            std::cerr << "WARNING: Failed to capture valid frame (null pointer or no image data). Waiting..." << std::endl; // More specific message
            usleep(10000);                                                                                                  // Wait if frame not ready
            continue;
        }

        // --- Send Frame Data to Python Script ---
        // std::cout << "DEBUG: Sending frame data..." << std::endl; // Can be noisy
        if (!SendFrameData(current_cam_rgb_frame))
        {
            // SendFrameData prints error message on failure
            std::cerr << "ERROR: Failed to send frame data. Exiting loop." << std::endl;
            break; // Exit loop on send error (likely connection closed)
        }
        // std::cout << "DEBUG: Frame data sent." << std::endl; // Can be noisy

        // --- Receive and Parse Detection Results ---
        // std::cout << "DEBUG: Receiving detection results..." << std::endl; // Can be noisy
        std::vector<ParsedDetection> detections = ReceiveDetectionResults();
        // std::cout << "DEBUG: Detection results received." << std::endl; // Can be noisy

        // Check if ReceiveDetectionResults indicated a connection error by returning empty
        // If it returned empty due to a connection error, the next SendFrameData
        // will likely fail, leading to a break.
        if (detections.empty() && client_socket_ < 0)
        {
            std::cerr << "ERROR: Socket error during detection results reception. Exiting loop." << std::endl;
            break;
        }

        // Copy original camera frame to the display frame for drawing
        // std::cout << "DEBUG: Copying frame for drawing..." << std::endl; // Can be noisy
        if (rgb_display_frame_ && current_cam_rgb_frame && rgb_display_frame_->m_ImageData && current_cam_rgb_frame->m_ImageData)
        {
            memcpy(rgb_display_frame_->m_ImageData, current_cam_rgb_frame->m_ImageData,
                   current_cam_rgb_frame->m_NumberOfPixels * current_cam_rgb_frame->m_PixelSize);
            // std::cout << "DEBUG: Frame copied." << std::endl; // Can be noisy
        }
        else
        {
            std::cerr << "WARNING: Cannot copy frame for drawing due to invalid pointers." << std::endl;
        }

        // --- Draw Detections ---
        // std::cout << "DEBUG: Drawing detections..." << std::endl; // Can be noisy
        for (const auto &det : detections)
        {
            DrawBoundingBox(rgb_display_frame_, det); // Draw on the *original resolution* display frame
        }
        // std::cout << "DEBUG: Detections drawn." << std::endl; // Can be noisy

        // --- Update Head Tracking based on Detections ---
        // std::cout << "DEBUG: Updating head tracking..." << std::endl; // Can be noisy
        UpdateHeadTracking(detections);
        // std::cout << "DEBUG: Head tracking updated." << std::endl; // Can be noisy

        // --- Stream Image ---
        // std::cout << "DEBUG: Streaming image..." << std::endl; // Can be noisy
        if (streamer_ && rgb_display_frame_)
        {
            streamer_->send_image(rgb_display_frame_);
            // std::cout << "DEBUG: Image streamed." << std::endl; // Can be noisy
        }
        else
        {
            std::cerr << "WARNING: Cannot stream image due to invalid streamer or frame pointer." << std::endl;
        }

        // usleep(10000); // Optional delay
    }

    std::cout << "INFO: HeadTracking main loop terminated." << std::endl;
}

void HeadTracking::Cleanup()
{
    std::cout << "INFO: Cleaning up HeadTracking resources..." << std::endl;

    if (client_socket_ >= 0)
    {
        close(client_socket_); // Close the client connection socket
        client_socket_ = -1;
    }
    // The server socket was closed after accepting the connection in InitializeSocketServer.
    // The socket file is unlinked in main.cpp.

    if (streamer_)
    {
        delete streamer_;
        streamer_ = nullptr;
    }

    if (rgb_display_frame_)
    {
        delete rgb_display_frame_;
        rgb_display_frame_ = nullptr;
    }

    std::cout << "INFO: HeadTracking cleanup complete." << std::endl;
}

// --- Private Helper Method Implementations ---

int HeadTracking::InitializeSocketServer()
{
    int server_sock;
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
    int client_sock = accept(server_sock, NULL, NULL);
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

bool HeadTracking::InitializeStreamer()
{
    std::cout << "INFO: Initializing MJPG streamer..." << std::endl;
    streamer_ = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);
    if (!streamer_)
    {
        std::cerr << "ERROR: Failed to create MJPG streamer." << std::endl;
        return false;
    }
    std::cout << "INFO: MJPG streamer initialized." << std::endl;
    return true;
}

bool HeadTracking::SendFrameData(Image *frame)
{
    if (!frame || !frame->m_ImageData || client_socket_ < 0)
    {
        // std::cerr << "ERROR: SendFrameData called with invalid parameters or socket not connected." << std::endl; // Can be noisy
        return false;
    }

    int frame_width = frame->m_Width;
    int frame_height = frame->m_Height;
    size_t frame_data_size = frame_width * frame_height * frame->m_PixelSize;

    // Send width and height first (as 4-byte integers)
    if (send(client_socket_, &frame_width, sizeof(frame_width), 0) < 0 ||
        send(client_socket_, &frame_height, sizeof(frame_height), 0) < 0)
    {
        std::cerr << "ERROR: Failed to send frame dimensions: " << strerror(errno) << std::endl;
        return false;
    }

    // Send the raw image data
    if (send(client_socket_, frame->m_ImageData, frame_data_size, 0) < 0)
    {
        std::cerr << "ERROR: Failed to send frame data: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

std::vector<ParsedDetection> HeadTracking::ReceiveDetectionResults()
{
    std::string detection_output;
    uint32_t result_size = 0;

    if (client_socket_ < 0)
    {
        std::cerr << "ERROR: ReceiveDetectionResults called with socket not connected." << std::endl;
        return {};
    }

    // Receive the size of the detection string
    std::string size_data = ReceiveExact(client_socket_, sizeof(result_size));
    if (size_data.empty())
    {
        // ReceiveExact already printed error/connection closed message
        client_socket_ = -1; // Mark socket as invalid
        return {};           // Return empty vector on error or closed connection
    }
    memcpy(&result_size, size_data.data(), sizeof(result_size));

    // std::cout << "DEBUG: Received result size: " << result_size << std::endl; // Debug received size

    // Receive the actual detection string
    if (result_size > 0)
    {
        detection_output = ReceiveExact(client_socket_, result_size);
        if (detection_output.empty())
        {
            // ReceiveExact already printed error/connection closed message
            client_socket_ = -1; // Mark socket as invalid
            return {};           // Return empty vector on error
        }
    }
    else
    {
        // Received size 0, means no detections or empty string sent
        // std::cout << "DEBUG: Received empty detection results string." << std::endl; // Debug empty results
        return {};
    }

    // std::cout << "DEBUG: Received detection output: '" << detection_output << "'" << std::endl; // Debug received string

    // Parse and return detections
    return ParseDetectionOutput(detection_output);
}

std::vector<ParsedDetection> HeadTracking::ParseDetectionOutput(const std::string &output)
{
    std::vector<ParsedDetection> detections;
    if (output.empty())
        return detections; // Return empty if input is empty

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
        else
        {
            std::cerr << "WARNING: Failed to parse detection line: '" << line << "'" << std::endl;
        }
    }
    return detections;
}

void HeadTracking::DrawBoundingBox(Image *image, const ParsedDetection &detection)
{
    if (!image || !image->m_ImageData)
    {
        std::cerr << "WARNING: DrawBoundingBox called with invalid image pointer." << std::endl;
        return;
    }

    int img_width = image->m_Width;
    int img_height = image->m_Height;
    int pixel_size = image->m_PixelSize; // Use pixel size from the Image object

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
    for (int x = xmin; x <= xmax; ++x)
    {
        if (ymin >= 0 && ymin < img_height)
        {
            unsigned char *p = &image->m_ImageData[(ymin * img_width + x) * pixel_size];
            p[0] = r;
            p[1] = g;
            p[2] = b;
        }
        if (ymax >= 0 && ymax < img_height)
        {
            unsigned char *p = &image->m_ImageData[(ymax * img_width + x) * pixel_size];
            p[0] = r;
            p[1] = g;
            p[2] = b;
        }
    }
    for (int y = ymin; y <= ymax; ++y)
    {
        if (xmin >= 0 && xmin < img_width)
        {
            unsigned char *p = &image->m_ImageData[(y * img_width + xmin) * pixel_size];
            p[0] = r;
            p[1] = g;
            p[2] = b;
        }
        if (xmax >= 0 && xmax < img_width)
        {
            unsigned char *p = &image->m_ImageData[(y * img_width + xmax) * pixel_size];
            p[0] = r;
            p[1] = g;
            p[2] = b;
        }
    }

    // For simplicity, printing to console:
    printf("Detected: %s (%.2f) at [%d,%d]-[%d,%d]\n",
           detection.label.c_str(), detection.score, xmin, ymin, xmax, ymax);

    // Drawing text label is more complex and omitted here.
}

void HeadTracking::UpdateHeadTracking(const std::vector<ParsedDetection> &detections)
{
    bool person_found_in_frame = false;
    Point2D tracked_object_center_for_head;
    std::string primary_detected_label = "none"; // Default to none

    // Find the first "person" detection (or the largest/closest if needed)
    for (const auto &det : detections)
    {
        if (det.label == "person")
        {
            // Eye color - magenta
            if (cm730_)
            {
                // Using MakeColor for the eye LEDs which expect 15-bit color
                cm730_->WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730_->MakeColor(255, 0, 255), 0); // Magenta
            }

            // Calculate center of the bounding box in original image pixel coordinates
            tracked_object_center_for_head.X = (det.xmin + det.xmax) / 2.0 * Camera::WIDTH;
            tracked_object_center_for_head.Y = (det.ymin + det.ymax) / 2.0 * Camera::HEIGHT;
            person_found_in_frame = true;       // A person was found in this frame
            primary_detected_label = det.label; // Store the label of the tracked object
            // For simplicity, track the first person found.
            // For better tracking, you might want to track the largest or closest person.
            break; // Stop searching after finding the first person
        }
    }

    // --- Update the shared detected label and center coordinates ---
    // Using a mutex is good practice for shared data, but for a single string
    // being written by one thread and read by another, it might be optional
    // depending on how critical it is to always get the absolute latest value
    // without any potential tearing (though string assignment is usually atomic).
    // std::lock_guard<std::mutex> lock(label_mutex_); // If using a mutex
    current_detected_label_ = primary_detected_label;
    current_tracked_object_center_ = tracked_object_center_for_head; // Update the shared center coordinates

    // --- Head Tracking State Management ---
    if (person_found_in_frame)
    {
        no_target_count_ = 0; // Reset the counter since a target was found
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
        if (std::abs(P_err.X) < pan_deadband_deg_)
        {
            P_err.X = 0.0; // Set error to zero within the deadband
        }
        else
        {
            // Apply additional scaling outside the deadband
            P_err.X *= pan_error_scale_;
        }

        if (std::abs(P_err.Y) < tilt_deadband_deg_)
        {
            P_err.Y = 0.0; // Set error to zero within the deadband
        }
        else
        {
            // Apply additional scaling outside the deadband
            P_err.Y *= tilt_error_scale_;
        }

        // Pass angular error to MoveTracking
        Head::GetInstance()->MoveTracking(P_err); // Actively track the person
    }
    else // No person found in the current frame
    {
        if (no_target_count_ < NO_TARGET_MAX_COUNT)
        {
            // Continue tracking based on the last known position or stop active tracking
            // head_module_->MoveTracking(); // Original BallTracker behavior (might hold last pos)

            Head::GetInstance()->MoveTracking(Point2D(0.0, 0.0)); // Alternative: stop active tracking and center head slowly

            no_target_count_++; // Increment counter
        }
        else
        {
            if (cm730_)
            {
                // Using WriteWord for the eye LEDs which expect 15-bit color value
                cm730_->WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, black_color_, NULL); // Black
            }

            // No target for too long, return to initial position
            Head::GetInstance()->MoveToHome(); // Alternative: stop active tracking and center head slowl

            // no_target_count_ remains at or above NO_TARGET_MAX_COUNT
        }
    }
}

std::string HeadTracking::ReceiveExact(int sock_fd, size_t num_bytes)
{
    std::string buffer(num_bytes, '\0');
    size_t total_received = 0;
    while (total_received < num_bytes)
    {
        ssize_t bytes_read = recv(sock_fd, &buffer[total_received], num_bytes - total_received, 0);
        if (bytes_read <= 0)
        {
            if (bytes_read == 0)
            {
                // Connection closed
                std::cerr << "ERROR: Socket connection closed by peer." << std::endl;
            }
            else
            {
                // Error receiving data
                std::cerr << "ERROR: Failed to receive data from socket: " << strerror(errno) << std::endl;
            }
            return ""; // Return empty string on error or disconnect
        }
        total_received += bytes_read;
    }
    return buffer;
}

// Public getter for the detected label
std::string HeadTracking::GetDetectedLabel()
{
    // If using a mutex, lock here before accessing current_detected_label_
    // std::lock_guard<std::mutex> lock(label_mutex_);
    return current_detected_label_;
}

// Public getter for the tracked object's center coordinates
Point2D HeadTracking::GetTrackedObjectCenter()
{
    // If using a mutex, lock here before accessing current_tracked_object_center_
    // std::lock_guard<std::mutex> lock(center_mutex_);
    return current_tracked_object_center_;
}
