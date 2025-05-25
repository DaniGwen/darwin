/*
 * HeadTracking.cpp
 *
 * Created on: May 17, 2025
 * Author: Your Name
 * Description: Implementation of the HeadTracking singleton class.
 * Directly controls head motors without relying on Head.cpp.
 */

#include "HeadTracking.h" // Include the corrected header first
#include <iostream>       // Explicitly include iostream for std::cout, std::cerr, std::endl
#include <string>         // Explicitly include string
#include <vector>         // Explicitly include vector
#include <sstream>        // Include sstream where stringstream is used
#include <cmath>          // For std::abs, round
#include <cstring>        // For memcpy
#include <cstdio>         // For printf (used in DrawBoundingBox)
#include <unistd.h>       // For usleep
#include <cstdlib>        // For system()
#include <mutex>          // For std::mutex (optional, but good practice for shared data)
#include <algorithm>      // For std::max, std::min

// Headers for Unix Domain Sockets
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h> // For errno and strerror

// Include LinuxCamera if it's not pulled in by Camera.h or LinuxDARwIn.h
#include "LinuxCamera.h" // Needed for LinuxCamera::GetInstance()
#include "MotionManager.h"

// Define socket path here (only once)
const char *SOCKET_PATH = "/tmp/darwin_detector.sock";

// --- Python Script Configuration ---
// IMPORTANT: Set the correct path to your Python detector script
const char *PYTHON_SCRIPT_PATH = "/home/darwin/darwin/aiy-maker-kit/examples/custom_detect_objects.py";

// --- MX-28 Motor Conversion Constants ---
// These constants are crucial for converting between degrees and the raw motor position values (0-4095).
// MX-28 motors have a range of 300 degrees (from -150 to +150 relative to center).
// The full range of 4096 units maps to 360 degrees, but the usable range is 300 degrees.
// A common mapping for MX-28: 2048 is center (0 degrees), 0 is -150 degrees, 4095 is +150 degrees.
// So, 1 degree = (4095 - 0) / 300 degrees = 13.65 units/degree.
// Let's use the standard 0.088 degrees per unit or 11.375 units per degree for 360 degrees range.
// However, for the 300 degree range, it's 4096 / 300 = 13.65 units/degree.
// The DXL SDK often uses 0.088 degrees/unit (360/4096). Let's use this for consistency if the
// angles are relative to the motor's full 360-degree resolution.
// Assuming 2048 is the mechanical center (0 degrees relative to motor).
const double MX28_CENTER_VALUE = 2048.0;
const double MX28_DEGREE_PER_UNIT = 360.0 / 4096.0; // 0.08789 degrees per unit
const double MX28_UNIT_PER_DEGREE = 4096.0 / 360.0; // 11.3777 units per degree

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
      m_PanAngle(0.0),
      m_TiltAngle(0.0),
      m_Pan_err(1.0),
      m_Pan_err_diff(0.5),
      m_Tilt_err(1.0),
      m_Tilt_err_diff(0.5),
      // Set very conservative default P and D gains here.
      // These will be overridden by INI settings if they exist.
      // The INI values are the ones you need to tune.
      // Start with very small values (e.g., 0.01 to 0.1 for P-gain, and even smaller for D-gain like 0.001 to 0.05)
      // and gradually increase them until you get smooth tracking without oscillation.
      m_Pan_p_gain(0.2),   // Starting point for P-gain (adjust in INI)
      m_Pan_d_gain(0.75),  // Starting point for D-gain (adjust in INI)
      m_Tilt_p_gain(0.2),  // Starting point for P-gain (adjust in INI)
      m_Tilt_d_gain(0.75), // Starting point for D-gain (adjust in INI)
      m_LeftLimit(80.0),
      m_RightLimit(-80.0),
      m_TopLimit(0.0),      // Will be set by Kinematics::EYE_TILT_OFFSET_ANGLE
      m_BottomLimit(-68.0), // Will be set by Kinematics::EYE_TILT_OFFSET_ANGLE
      m_Pan_Home(0.0),
      m_Tilt_Home(-30.0), // Will be set by Kinematics::EYE_TILT_OFFSET_ANGLE
      no_target_count_(0),
      // These scales multiply the error *before* applying P/D gains.
      // Keep them at 1.0 unless you have a specific reason to scale the error itself.
      pan_error_scale_(1.0),
      tilt_error_scale_(1.0),
      pan_deadband_deg_(0.05),
      tilt_deadband_deg_(0.05),
      black_color_(0),
      frame_counter_(0),
      current_detected_label_("none"),
      current_tracked_object_center_(0.0, 0.0)
{
    // Constructor is intentionally minimal.
    // Initialization that might fail or requires external resources
    // should be done in the Initialize() method.
}

// Destructor definition
HeadTracking::~HeadTracking()
{
    Cleanup();
    // ini_settings_ and cm730_ are not owned by HeadTracking
}

// Modified Initialize signature to accept CM730* directly
bool HeadTracking::Initialize(minIni *ini, CM730 *cm730)
{
    ini_settings_ = ini;
    cm730_ = cm730;

    // Basic check if CM730 pointer is valid
    if (!cm730_)
    {
        std::cerr << "ERROR: HeadTracking initialization failed: Invalid CM730 pointer passed." << std::endl;
        return false;
    }

    // --- Initialize Components ---

    // 0. Auto-start the Python detector script
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

    // 2. Initialize MJPG Streamer
    if (!InitializeStreamer())
    {
        std::cerr << "ERROR: HeadTracking initialization failed: Streamer setup failed." << std::endl;
        Cleanup(); // Clean up already initialized resources
        return false;
    }

    // 3. Load Head-specific settings from INI
    LoadHeadSettings(ini_settings_);

      // Explicitly enable head joints and set initial gains
    cm730_->WriteByte(JointData::ID_HEAD_PAN, MX28::P_TORQUE_ENABLE, 1, 0);  // Enable torque for Pan
    cm730_->WriteByte(JointData::ID_HEAD_TILT, MX28::P_TORQUE_ENABLE, 1, 0); // Enable torque for Tilt

    // Set P and D gains using values loaded from INI
    cm730_->WriteByte(JointData::ID_HEAD_PAN, MX28::P_P_GAIN, 8, 0);
    cm730_->WriteByte(JointData::ID_HEAD_TILT, MX28::P_P_GAIN, 8, 0);

    // 5. Create display frame buffer
    rgb_display_frame_ = new Robot::Image(Camera::WIDTH, Camera::HEIGHT, Robot::Image::RGB_PIXEL_SIZE);
    if (!rgb_display_frame_)
    {
        std::cerr << "ERROR: HeadTracking initialization failed: Failed to create display frame buffer." << std::endl;
        Cleanup();
        return false;
    }

    // 6. Load HeadTracking tuning parameters from INI if available
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
    if (client_socket_ < 0 || !streamer_ || !rgb_display_frame_ || !ini_settings_ || !cm730_)
    {
        std::cerr << "ERROR: HeadTracking not fully initialized. Cannot run." << std::endl;
        return;
    }

    std::cout << "INFO: Starting HeadTracking main loop..." << std::endl;

    usleep(500000); // 0.5 second delay

    const int STATUS_CHECK_INTERVAL = 30; // Check status every 30 frames

    while (1)
    {
        // --- Capture Frame ---
        LinuxCamera::GetInstance()->CaptureFrame();
        Robot::Image *current_cam_rgb_frame = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;

        if (!current_cam_rgb_frame || !current_cam_rgb_frame->m_ImageData)
        {
            std::cerr << "WARNING: Failed to capture valid frame (null pointer or no image data). Waiting..." << std::endl;
            usleep(10000);
            continue;
        }

        // --- Send Frame Data to Python Script ---
        if (!SendFrameData(current_cam_rgb_frame))
        {
            // SendFrameData prints error message
            std::cerr << "ERROR: Failed to send frame data. Exiting loop." << std::endl;
            break; // Exit loop on send error (likely connection closed)
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

        // --- Apply calculated angles to motors ---
        ApplyHeadAngles();

        // --- Stream Image ---
        if (streamer_ && rgb_display_frame_)
        {
            streamer_->send_image(rgb_display_frame_);
        }
        else
        {
            std::cerr << "WARNING: Cannot stream image due to invalid streamer or frame pointer." << std::endl;
        }
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

    // cm730_ is not owned here, it is managed externally.

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

bool HeadTracking::SendFrameData(Robot::Image *frame)
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
        ParsedDetection det;

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

void HeadTracking::DrawBoundingBox(Robot::Image *image, const ParsedDetection &detection)
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

    printf("Detected: %s (%.2f) at [%d,%d]-[%d,%d]\n",
           detection.label.c_str(), detection.score, xmin, ymin, xmax, ymax);
}

void HeadTracking::UpdateHeadTracking(const std::vector<ParsedDetection> &detections)
{
    bool person_found_in_frame = false;
    Robot::Point2D tracked_object_center_for_head;
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
        Robot::Point2D P_err;

        Robot::Point2D pixel_offset_from_center;
        pixel_offset_from_center.X = tracked_object_center_for_head.X - (Camera::WIDTH / 2.0);
        pixel_offset_from_center.Y = tracked_object_center_for_head.Y - (Camera::HEIGHT / 2.0);

        std::cout << "DEBUG: Raw Pixel Offset - X: " << pixel_offset_from_center.X
                  << ", Y: " << pixel_offset_from_center.Y << std::endl;

        pixel_offset_from_center.X *= -1; // Invert X for pan (left is positive angle)
        pixel_offset_from_center.Y *= -1; // Invert Y for tilt (up is positive angle)

        P_err.X = pixel_offset_from_center.X * (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH);
        P_err.Y = pixel_offset_from_center.Y * (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT);

        std::cout << "DEBUG: Raw Angular Error (deg) - Pan: " << P_err.X
                  << ", Tilt: " << P_err.Y << std::endl;

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

        UpdateHeadAngles(P_err); // Call the new method to update angles
        std::cout << "DEBUG: Head Moving: P_err.X=" << P_err.X << ", P_err.Y=" << P_err.Y << std::endl;
    }
    else
    {
        if (no_target_count_ < NO_TARGET_MAX_COUNT)
        {
            // Slowly center head if no target for a few frames
            // This applies a small, fixed correction towards home.
            double pan_center_speed = (m_Pan_Home - m_PanAngle) * 0.05;    // 5% of distance to home
            double tilt_center_speed = (m_Tilt_Home - m_TiltAngle) * 0.05; // 5% of distance to home
            UpdateHeadAngles(Robot::Point2D(pan_center_speed, tilt_center_speed));
            std::cout << "DEBUG: Head Centering: NoTargetCount=" << no_target_count_ << std::endl;
            no_target_count_++;
        }
        else
        {
            if (cm730_)
            {
                cm730_->WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, black_color_, NULL); // Black
            }
            MoveToHome(); // Return to initial position
            std::cout << "DEBUG: Head Moving to Home." << std::endl;
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

Robot::Point2D HeadTracking::GetTrackedObjectCenter()
{
    return current_tracked_object_center_;
}

// --- New methods to replace Head.cpp functionality and for angle conversion ---

// Converts an angle in degrees to a raw MX-28 motor value (0-4095)
// Assumes 0 degrees is the center position (2048).
// Positive angles move towards higher values, negative towards lower.
int HeadTracking::Deg2Value(double angle_deg)
{
    // Calculate the raw value relative to the center position
    double value = MX28_CENTER_VALUE + (angle_deg * MX28_UNIT_PER_DEGREE);

    // Clamp the value to the valid MX-28 range (0-4095)
    value = std::max(0.0, std::min(4095.0, value));

    return static_cast<int>(round(value));
}

// Converts a raw MX-28 motor value (0-4095) to an angle in degrees
// Assumes 2048 is the center position (0 degrees).
double HeadTracking::Value2Deg(int value)
{
    return (value - MX28_CENTER_VALUE) * MX28_DEGREE_PER_UNIT;
}

void HeadTracking::LoadHeadSettings(minIni *ini)
{
    // Load P and D gains from INI. These are the values that directly control motor responsiveness.
    // If the head is jerking, these values in your config.ini are too high.
    // Start with very small values and gradually increase them.
    m_Pan_p_gain = ini->getd("Head Pan/Tilt", "pan_p_gain", m_Pan_p_gain);
    m_Pan_d_gain = ini->getd("Head Pan/Tilt", "pan_d_gain", m_Pan_d_gain); // Corrected ini key for consistency
    m_Tilt_p_gain = ini->getd("Head Pan/Tilt", "tilt_p_gain", m_Tilt_p_gain);
    m_Tilt_d_gain = ini->getd("Head Pan/Tilt", "tilt_d_gain", m_Tilt_d_gain);

    // Load angle limits in degrees
    m_LeftLimit = ini->getd("Head Pan/Tilt", "left_limit", m_LeftLimit);
    m_RightLimit = ini->getd("Head Pan/Tilt", "right_limit", m_RightLimit);
    m_TopLimit = ini->getd("Head Pan/Tilt", "top_limit", Kinematics::EYE_TILT_OFFSET_ANGLE);
    m_BottomLimit = ini->getd("Head Pan/Tilt", "bottom_limit", Kinematics::EYE_TILT_OFFSET_ANGLE - 65.0);

    // Load home positions in degrees
    m_Pan_Home = ini->getd("Head Pan/Tilt", "pan_home", 0.0);
    m_Tilt_Home = ini->getd("Head Pan/Tilt", "tilt_home", Kinematics::EYE_TILT_OFFSET_ANGLE - 30.0);

    std::cout << "INFO: HeadTracking::LoadHeadSettings - Pan_P_GAIN: " << m_Pan_p_gain
              << ", Pan_D_GAIN: " << m_Pan_d_gain
              << ", Tilt_P_GAIN: " << m_Tilt_p_gain
              << ", Tilt_D_GAIN: " << m_Tilt_d_gain << std::endl;
    std::cout << "INFO: HeadTracking::LoadHeadSettings - Limits: L=" << m_LeftLimit << ", R=" << m_RightLimit
              << ", T=" << m_TopLimit << ", B=" << m_BottomLimit << std::endl;
    std::cout << "INFO: HeadTracking::LoadHeadSettings - Home: Pan=" << m_Pan_Home << ", Tilt=" << m_Tilt_Home << std::endl;
}

void HeadTracking::CheckLimit()
{
    // Clamp the angles to the defined limits
    m_PanAngle = std::max(m_RightLimit, std::min(m_LeftLimit, m_PanAngle));
    m_TiltAngle = std::max(m_BottomLimit, std::min(m_TopLimit, m_TiltAngle));

    std::cout << "DEBUG: HeadTracking::CheckLimit - PanAngle (clamped): " << m_PanAngle << ", TiltAngle (clamped): " << m_TiltAngle << std::endl;
}

void HeadTracking::MoveToHome()
{
    MoveByAngle(m_Pan_Home, m_Tilt_Home);
    std::cout << "DEBUG: HeadTracking::MoveToHome - Moving to Pan_Home: " << m_Pan_Home << ", Tilt_Home: " << m_Tilt_Home << std::endl;
}

void HeadTracking::MoveByAngle(double pan, double tilt)
{
    m_PanAngle = pan;
    m_TiltAngle = tilt;

    CheckLimit(); // Apply limits after setting new angles
    std::cout << "DEBUG: HeadTracking::MoveByAngle - Target Pan: " << pan << ", Target Tilt: " << tilt
              << " (After Limit: Pan: " << m_PanAngle << ", Tilt: " << m_TiltAngle << ")" << std::endl;
}

void HeadTracking::MoveByAngleOffset(double pan_offset, double tilt_offset)
{
    // Apply offset to current angles and then move to the new absolute angles
    MoveByAngle(m_PanAngle + pan_offset, m_TiltAngle + tilt_offset);
    std::cout << "DEBUG: HeadTracking::MoveByAngleOffset - Offset Pan: " << pan_offset << ", Offset Tilt: " << tilt_offset << std::endl;
}

void HeadTracking::InitTracking()
{
    m_Pan_err = 0;
    m_Pan_err_diff = 0;
    m_Tilt_err = 0;
    m_Tilt_err_diff = 0;
    std::cout << "DEBUG: HeadTracking::InitTracking - Tracking errors reset." << std::endl;
}

void HeadTracking::UpdateHeadAngles(Robot::Point2D err)
{
    // Calculate the derivative error (change in error)
    m_Pan_err_diff = err.X - m_Pan_err;
    m_Pan_err = err.X; // Update current error

    m_Tilt_err_diff = err.Y - m_Tilt_err;
    m_Tilt_err = err.Y; // Update current error

    std::cout << "DEBUG: HeadTracking::UpdateHeadAngles - Input P_err.X: " << err.X << ", P_err.Y: " << err.Y << std::endl;
    std::cout << "DEBUG: HeadTracking::UpdateHeadAngles - m_Pan_err: " << m_Pan_err << ", m_Pan_err_diff: " << m_Pan_err_diff << std::endl;
    std::cout << "DEBUG: HeadTracking::UpdateHeadAngles - m_Tilt_err: " << m_Tilt_err << ", m_Tilt_err_diff: " << m_Tilt_err_diff << std::endl;

    double pOffset, dOffset;

    // Calculate PID components and update target angles
    // The `pan_error_scale_` and `tilt_error_scale_` (loaded from INI or default 1.0)
    // are applied to the raw angular error (P_err.X, P_err.Y) before PID calculation.
    // The m_Pan_p_gain, m_Pan_d_gain (loaded from INI) are the actual PID gains.
    // If the head is jerking, reduce the P_GAIN and D_GAIN values in your config.ini.
    pOffset = m_Pan_err * m_Pan_p_gain;
    dOffset = m_Pan_err_diff * m_Pan_d_gain;
    m_PanAngle += (pOffset + dOffset); // Update target pan angle

    pOffset = m_Tilt_err * m_Tilt_p_gain;
    dOffset = m_Tilt_err_diff * m_Tilt_d_gain;
    m_TiltAngle += (pOffset + dOffset); // Update target tilt angle

    std::cout << "DEBUG: HeadTracking::UpdateHeadAngles - Pan pOffset: " << m_Pan_err * m_Pan_p_gain
              << ", Pan dOffset: " << m_Pan_err_diff * m_Pan_d_gain
              << ", New PanAngle (before limit): " << m_PanAngle << std::endl;
    std::cout << "DEBUG: HeadTracking::UpdateHeadAngles - Tilt pOffset: " << m_Tilt_err * m_Tilt_p_gain
              << ", Tilt dOffset: " << m_Tilt_err_diff * m_Tilt_d_gain
              << ", New TiltAngle (before limit): " << m_TiltAngle << std::endl;

    CheckLimit(); // Ensure the updated angles are within limits
}

void HeadTracking::ApplyHeadAngles()
{
    if (cm730_)
    {
        int pan_torque_enable = 0;
        int tilt_torque_enable = 0;

        // Read current torque status
        cm730_->ReadByte(JointData::ID_HEAD_PAN, MX28::P_TORQUE_ENABLE, &pan_torque_enable, 0);
        cm730_->ReadByte(JointData::ID_HEAD_TILT, MX28::P_TORQUE_ENABLE, &tilt_torque_enable, 0);

        if (pan_torque_enable == 1 && tilt_torque_enable == 1)
        {
            // Convert the calculated angles (in degrees) to MX-28 motor values (0-4095)
            int pan_goal_value = Deg2Value(m_PanAngle);
            int tilt_goal_value = Deg2Value(m_TiltAngle);

            // Write the converted values to the motor goal position registers
            cm730_->WriteWord(JointData::ID_HEAD_PAN, MX28::P_GOAL_POSITION_L, pan_goal_value, 0);
            cm730_->WriteWord(JointData::ID_HEAD_TILT, MX28::P_GOAL_POSITION_L, tilt_goal_value, 0);

            std::cout
                << "DEBUG: HeadTracking::ApplyHeadAngles - Setting Pan Deg: " << m_PanAngle
                << " (Value: " << pan_goal_value << "), Tilt Deg: " << m_TiltAngle
                << " (Value: " << tilt_goal_value << ")" << std::endl;
        }
        else
        {
            std::cout << "DEBUG: HeadTracking::ApplyHeadAngles - Head joints are not enabled, skipping angle setting." << std::endl;
        }
    }
    else
    {
        std::cerr << "ERROR: HeadTracking::ApplyHeadAngles - CM730 not initialized, cannot set angles." << std::endl;
    }
}
