/*
 * HeadTracking.cpp
 *
 * Created on: May 17, 2025
 * Author: Your Name
 * Description: Implementation of the HeadTracking singleton class.
 * Receives initialized Motion Framework (Head, CM730) pointers.
 * Added explicit includes for stringstream fix.
 * Defined SOCKET_PATH here.
 * Added detailed logging to Run() loop, including pre-capture log and initial delay.
 * Added Python script startup to Initialize().
 * Added CM730 pointer and LED color control logic.
 * Added debug logging for LED commands and a basic LED test.
 * Added member variable and getter to expose the detected label.
 * Added member variable and getter to expose the tracked object's center coordinates.
 * Head motor control is now handled directly by HeadTracking, not MotionManager.
 */

#include "HeadTracking.h" // Include the corrected header first
#include <iostream> // Explicitly include iostream for std::cout, std::cerr, std::endl
#include <string>   // Explicitly include string
#include <vector>   // Explicitly include vector
#include <sstream>  // Include sstream where stringstream is used
#include <cmath>    // For std::abs
#include <cstring>  // For memcpy
#include <cstdio>   // For printf (used in DrawBoundingBox)
#include <unistd.h> // For usleep
#include <cstdlib>  // For system()
#include <mutex>    // For std::mutex (optional, but good practice for shared data)

// Headers for Unix Domain Sockets
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h> // For errno and strerror

// Include LinuxCamera if it's not pulled in by Camera.h or LinuxDARwIn.h
#include "LinuxCamera.h" // Needed for LinuxCamera::GetInstance()

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
      head_module_(nullptr), // Initialize head_module_
      cm730_(nullptr),       // Initialize cm7730_
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

// Destructor definition (must be defined if declared)
HeadTracking::~HeadTracking()
{
    Cleanup();
    // Note: ini_settings_ is owned by main and should not be deleted here.
    // Motion framework components (head_module_, cm730_) are not owned here.
}

// Modified Initialize signature to accept Head*
bool HeadTracking::Initialize(minIni *ini, Robot::Head *head_module, CM730 *cm730)
{
    ini_settings_ = ini;
    head_module_ = head_module; // Store passed pointer
    cm730_ = cm730;

    // Basic check if passed pointers are valid
    if (!head_module_ || !cm730_)
    {
        std::cerr << "ERROR: HeadTracking initialization failed: Invalid Head or CM730 pointer passed." << std::endl;
        return false;
    }

    // --- Initialize Components ---

    // 0. Auto-start the Python detector script (moved from main)
    std::string command = "python3 ";
    command += PYTHON_SCRIPT_PATH;
    command += " &"; // Run in background
    std::cout << "INFO: Starting Python detector script: " << command << std::endl;
    int system_return = system(command.c_str());

    if (system_return != 0)
    {
        std::cerr << "WARNING: Failed to start Python script using system(). Make sure the path is correct and python3 is in PATH." << std::endl;
    }
    usleep(2000000); // 2 second delay (adjust if needed) to allow script to start and create socket

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

    // 4. Configure Head Module (now directly controlled by HeadTracking)
    std::cout << "INFO: Configuring Head module directly via HeadTracking..." << std::endl;
    // Explicitly enable head joints and set initial gains
    head_module_->m_Joint.SetEnableHeadOnly(true, true); // Enable torque for head motors
    head_module_->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 8); // Set P-gain for pan
    head_module_->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 8); // Set P-gain for tilt
    std::cout << "INFO: Head module configured." << std::endl;

    // 5. Create display frame buffer
    rgb_display_frame_ = new Robot::Image(Camera::WIDTH, Camera::HEIGHT, Robot::Image::RGB_PIXEL_SIZE); // Use Robot::Image
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
    if (client_socket_ < 0 || !streamer_ || !rgb_display_frame_ || !ini_settings_ || !head_module_ || !cm730_)
    {
        std::cerr << "ERROR: HeadTracking not fully initialized. Cannot run." << std::endl;
        return;
    }

    std::cout << "INFO: Starting HeadTracking main loop..." << std::endl;

    usleep(500000); // 0.5 second delay

    while (1)
    {
        // --- Capture Frame ---
        LinuxCamera::GetInstance()->CaptureFrame();
        Robot::Image *current_cam_rgb_frame = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame; // Use Robot::Image

        if (!current_cam_rgb_frame || !current_cam_rgb_frame->m_ImageData)
        {
            std::cerr << "WARNING: Failed to capture valid frame (null pointer or no image data). Waiting..." << std::endl;
            usleep(10000);
            continue;
        }

        // --- Send Frame Data to Python Script ---
        if (!SendFrameData(current_cam_rgb_frame))
        {
            std::cerr << "ERROR: Failed to send frame data. Exiting loop." << std::endl;
            break;
        }

        // --- Receive and Parse Detection Results ---
        std::vector<ParsedDetection> detections = ReceiveDetectionResults();

        if (detections.empty() && client_socket_ < 0)
        {
            std::cerr << "ERROR: Socket error during detection results reception. Exiting loop." << std::endl;
            break;
        }

        // Copy original camera frame to the display frame for drawing
        if (rgb_display_frame_ && current_cam_rgb_frame && rgb_display_frame_->m_ImageData && current_cam_rgb_frame->m_ImageData)
        {
            memcpy(rgb_display_frame_->m_ImageData, current_cam_rgb_frame->m_ImageData,
                   current_cam_rgb_frame->m_NumberOfPixels * current_cam_rgb_frame->m_PixelSize);
        }
        else
        {
            std::cerr << "WARNING: Cannot copy frame for drawing due to invalid pointers." << std::endl;
        }

        // --- Draw Detections ---
        for (const auto &det : detections)
        {
            DrawBoundingBox(rgb_display_frame_, det);
        }

        // --- Update Head Tracking based on Detections ---
        UpdateHeadTracking(detections);

        // --- IMPORTANT: Manually process the Head module to apply new angles ---
        // Since Head is no longer added to MotionManager, we must call Process() here.
        if (head_module_) {
            head_module_->Process();
        } else {
            std::cerr << "ERROR: Head module is null in HeadTracking::Run(). Cannot process head movements." << std::endl;
        }


        // --- Stream Image ---
        if (streamer_ && rgb_display_frame_)
        {
            streamer_->send_image(rgb_display_frame_);
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

    // Head module and CM730 are not owned here, they are managed externally.

    std::cout << "INFO: HeadTracking cleanup complete." << std::endl;
}

// --- Private Helper Method Implementations ---

int HeadTracking::InitializeSocketServer()
{
    int server_sock;
    struct sockaddr_un server_addr;

    unlink(SOCKET_PATH); // Remove existing socket file

    server_sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_sock < 0)
    {
        std::cerr << "ERROR: Failed to create socket: " << strerror(errno) << std::endl;
        return -1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sun_family = AF_UNIX;
    strncpy(server_addr.sun_path, SOCKET_PATH, sizeof(server_addr.sun_path) - 1);

    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {
        std::cerr << "ERROR: Failed to bind socket to " << SOCKET_PATH << ": " << strerror(errno) << std::endl;
        close(server_sock);
        return -1;
    }

    if (listen(server_sock, 5) < 0)
    {
        std::cerr << "ERROR: Failed to listen on socket: " << strerror(errno) << std::endl;
        close(server_sock);
        return -1;
    }

    std::cout << "INFO: Waiting for Python detector script connection on " << SOCKET_PATH << "..." << std::endl;

    int client_sock = accept(server_sock, NULL, NULL);
    if (client_sock < 0)
    {
        std::cerr << "ERROR: Failed to accept client connection: " << strerror(errno) << std::endl;
        close(server_sock);
        return -1;
    }

    std::cout << "INFO: Python detector script connected." << std::endl;
    close(server_sock); // Close the listening socket

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

bool HeadTracking::SendFrameData(Robot::Image *frame) // Use Robot::Image
{
    if (!frame || !frame->m_ImageData || client_socket_ < 0)
    {
        return false;
    }

    int frame_width = frame->m_Width;
    int frame_height = frame->m_Height;
    size_t frame_data_size = frame_width * frame_height * frame->m_PixelSize;

    if (send(client_socket_, &frame_width, sizeof(frame_width), 0) < 0 ||
        send(client_socket_, &frame_height, sizeof(frame_height), 0) < 0)
    {
        std::cerr << "ERROR: Failed to send frame dimensions: " << strerror(errno) << std::endl;
        return false;
    }

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

    std::string size_data = ReceiveExact(client_socket_, sizeof(result_size));
    if (size_data.empty())
    {
        client_socket_ = -1;
        return {};
    }
    memcpy(&result_size, size_data.data(), sizeof(result_size));

    if (result_size > 0)
    {
        detection_output = ReceiveExact(client_socket_, result_size);
        if (detection_output.empty())
        {
            client_socket_ = -1;
            return {};
        }
    }
    else
    {
        return {};
    }
    return ParseDetectionOutput(detection_output);
}

std::vector<ParsedDetection> HeadTracking::ParseDetectionOutput(const std::string &output)
{
    std::vector<ParsedDetection> detections;
    if (output.empty())
        return detections;

    std::stringstream ss(output);
    std::string line;

    while (std::getline(ss, line, '\n'))
    {
        std::stringstream line_ss(line);
        ParsedDetection det; // Correctly declare ParsedDetection here

        std::string label_str;

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

void HeadTracking::DrawBoundingBox(Robot::Image *image, const ParsedDetection &detection) // Use Robot::Image
{
    if (!image || !image->m_ImageData)
    {
        std::cerr << "WARNING: DrawBoundingBox called with invalid image pointer." << std::endl;
        return;
    }

    int img_width = image->m_Width;
    int img_height = image->m_Height;
    int pixel_size = image->m_PixelSize;

    int xmin = static_cast<int>(detection.xmin * img_width);
    int ymin = static_cast<int>(detection.ymin * img_height);
    int xmax = static_cast<int>(detection.xmax * img_width);
    int ymax = static_cast<int>(detection.ymax * img_height);

    xmin = std::max(0, std::min(xmin, img_width - 1));
    ymin = std::max(0, std::min(ymin, img_height - 1));
    xmax = std::max(0, std::min(xmax, img_width - 1));
    ymax = std::max(0, std::min(ymax, img_height - 1));

    unsigned char r = 255, g = 0, b = 0;

    for (int x = xmin; x <= xmax; ++x)
    {
        if (ymin >= 0 && ymin < img_height)
        {
            unsigned char *p = &image->m_ImageData[(ymin * img_width + x) * pixel_size];
            p[0] = r; p[1] = g; p[2] = b;
        }
        if (ymax >= 0 && ymax < img_height)
        {
            unsigned char *p = &image->m_ImageData[(ymax * img_width + x) * pixel_size];
            p[0] = r; p[1] = g; p[2] = b;
        }
    }
    for (int y = ymin; y <= ymax; ++y)
    {
        if (xmin >= 0 && xmin < img_width)
        {
            unsigned char *p = &image->m_ImageData[(y * img_width + xmin) * pixel_size];
            p[0] = r; p[1] = g; p[2] = b;
        }
        if (xmax >= 0 && xmax < img_width)
        {
            unsigned char *p = &image->m_ImageData[(y * img_width + xmax) * pixel_size];
            p[0] = r; p[1] = g; p[2] = b;
        }
    }

    printf("Detected: %s (%.2f) at [%d,%d]-[%d,%d]\n",
           detection.label.c_str(), detection.score, xmin, ymin, xmax, ymax);
}

void HeadTracking::UpdateHeadTracking(const std::vector<ParsedDetection> &detections)
{
    bool person_found_in_frame = false;
    Robot::Point2D tracked_object_center_for_head; // Use Robot::Point2D
    std::string primary_detected_label = "none";

    for (const auto &det : detections)
    {
        if (det.label == "person")
        {
            if (cm730_)
            {
                cm730_->WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730_->MakeColor(255, 0, 255), 0); // Magenta
            }

            tracked_object_center_for_head.X = (det.xmin + det.xmax) / 2.0 * Camera::WIDTH;
            tracked_object_center_for_head.Y = (det.ymin + det.ymax) / 2.0 * Camera::HEIGHT;
            person_found_in_frame = true;
            primary_detected_label = det.label;
            break;
        }
    }

    current_detected_label_ = primary_detected_label;
    current_tracked_object_center_ = tracked_object_center_for_head;

    if (person_found_in_frame)
    {
        no_target_count_ = 0;
        Robot::Point2D P_err; // Use Robot::Point2D

        Robot::Point2D pixel_offset_from_center; // Use Robot::Point2D
        pixel_offset_from_center.X = tracked_object_center_for_head.X - (Camera::WIDTH / 2.0);
        pixel_offset_from_center.Y = tracked_object_center_for_head.Y - (Camera::HEIGHT / 2.0);

        pixel_offset_from_center.X *= -1;
        pixel_offset_from_center.Y *= -1; // Keep this inversion if it works for your robot's tilt

        P_err.X = pixel_offset_from_center.X * (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH);
        P_err.Y = pixel_offset_from_center.Y * (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT);

        if (std::abs(P_err.X) < pan_deadband_deg_)
        {
            P_err.X = 0.0;
        }
        else
        {
            P_err.X *= pan_error_scale_;
        }

        if (std::abs(P_err.Y) < tilt_deadband_deg_)
        {
            P_err.Y = 0.0;
        }
        else
        {
            P_err.Y *= tilt_error_scale_;
        }

        if (head_module_)
        {
            head_module_->MoveTracking(P_err);
        }
        else
        {
            std::cerr << "ERROR: Head module not initialized in UpdateHeadTracking." << std::endl;
        }
    }
    else
    {
        if (no_target_count_ < NO_TARGET_MAX_COUNT) // Correctly use NO_TARGET_MAX_COUNT
        {
            if (head_module_)
            {
                head_module_->MoveTracking(Robot::Point2D(0.0, 0.0)); // Use Robot::Point2D
            }
            else
            {
                std::cerr << "ERROR: Head module not initialized for centering in UpdateHeadTracking." << std::endl;
            }
            no_target_count_++;
        }
        else
        {
            if (cm730_)
            {
                cm730_->WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, black_color_, NULL); // Black
            }

            if (head_module_)
            {
                head_module_->MoveToHome(); // Return to initial position
            }
            else
            {
                std::cerr << "ERROR: Head module not initialized for MoveToHome in UpdateHeadTracking." << std::endl;
            }
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
                std::cerr << "ERROR: Socket connection closed by peer." << std::endl;
            }
            else
            {
                std::cerr << "ERROR: Failed to receive data from socket: " << strerror(errno) << std::endl;
            }
            return "";
        }
        total_received += bytes_read;
    }
    return buffer;
}

std::string HeadTracking::GetDetectedLabel()
{
    return current_detected_label_;
}

Robot::Point2D HeadTracking::GetTrackedObjectCenter() // Use Robot::Point2D
{
    return current_tracked_object_center_;
}
