/*
 * HeadTracking.h
 *
 * Created on: May 17, 2025
 * Author: Your Name
 * Description: Singleton class to manage head tracking, socket communication
 * with a detector script, and MJPG streaming for a DARwIn-OP robot.
 * Directly controls head motors without relying on Head.cpp.
 */

#ifndef HEADTRACKING_H_
#define HEADTRACKING_H_

#include <string>   // For std::string
#include <vector>   // For std::vector
#include <memory>   // For std::unique_ptr
#include <iostream> // For std::cerr, std::cout
#include <cmath>    // For std::abs (needed for CheckLimit, PID)
#include <mutex>
#include <map>

// Headers for Unix Domain Sockets
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h> // For errno and strerror

// Framework includes that are still necessary
#include "Camera.h"        // For Camera::WIDTH, Camera::HEIGHT, VIEW_H_ANGLE, VIEW_V_ANGLE
#include "mjpg_streamer.h" // For mjpg_streamer class
#include "CM730.h"         // For CM730 class (direct motor control)
#include "minIni.h"        // For minIni class
#include "Point.h"         // For Robot::Point2D
#include "JointData.h"     // For JointData::ID_HEAD_PAN, ID_HEAD_TILT, MX28 constants
#include "Kinematics.h"    // **ADDED: For Kinematics::EYE_TILT_OFFSET_ANGLE**

// Declare socket path using extern - the definition is in HeadTracking.cpp
extern const char *SOCKET_PATH;

namespace Robot
{
    class MotionManager; // Still forward declare if needed by other parts of the system
    class Image;         // For Robot::Image

    // Structure to hold parsed detection data received from Python
    struct ParsedDetection
    {
        std::string label;
        float score;
        float xmin, ymin, xmax, ymax; // Normalized coordinates [0.0, 1.0]
    };

    class HeadTracking
    {
    public:
        // Singleton access method
        static HeadTracking *GetInstance();
        static void DestroyInstance();

        // Initialization method
        // Now only takes minIni and CM730, as Head module is integrated.
        bool Initialize(minIni *ini, CM730 *cm730);

        // Main tracking loop
        void Run();

        void Cleanup();

        std::string GetDetectedLabel();
        Robot::Point2D GetTrackedObjectCenter();
        int GetDetectionScore();
        double GetDetectedObjectDistance() const;

        // Static methods to control tracking state
        static void SetTrackingEnabled(bool enable);
        static bool IsTrackingEnabled();

        void SetMotorCommandInterval(int interval_ms);
        int GetMotorCommandInterval() const;

        // Explicitly declare the destructor
        ~HeadTracking();

    private:
        // Private constructor to enforce singleton pattern
        HeadTracking();

        // Singleton instance
        static HeadTracking *m_UniqueInstance;
        static std::mutex m_Mutex;     // Mutex to protect shared resources (like the enable flag)
        static bool m_TrackingEnabled; // New flag to control tracking

        // Delete copy constructor and assignment operator
        HeadTracking(const HeadTracking &) = delete;
        HeadTracking &operator=(const HeadTracking &) = delete;

        // --- Member Variables (Ordered to match constructor for -Wreorder warning) ---
        int client_socket_;               // File descriptor for the client socket connection
        mjpg_streamer *streamer_;         // Pointer to the MJPG streamer instance
        minIni *ini_;                     // Pointer to loaded INI settings (owned by main)
        CM730 *cm730_;                    // Pointer to CM730 instance (direct motor control)
        Robot::Image *rgb_display_frame_; // Image buffer for the output frame with detections

        // Head control state variables (moved from Head.h)
        double m_PanAngle;
        double m_TiltAngle;
        double m_Pan_err;
        double m_Pan_err_diff;
        double m_Tilt_err;
        double m_Tilt_err_diff;

        // PID Gains (moved from Head.h)
        double m_Pan_p_gain;
        double m_Pan_d_gain;
        double m_Tilt_p_gain;
        double m_Tilt_d_gain;

        // Limits (moved from Head.h)
        double m_LeftLimit;
        double m_RightLimit;
        double m_TopLimit;
        double m_BottomLimit;

        // Home position (moved from Head.h)
        double m_Pan_Home;
        double m_Tilt_Home;

        // Tracking state variables
        int no_target_count_;
        static const int NO_TARGET_MAX_COUNT = 40; // Number of frames to wait before initiating scan

        // Tuning Parameters for Centering (loaded from INI in Initialize)
        double pan_error_scale_;
        double tilt_error_scale_;
        double pan_deadband_deg_;
        double tilt_deadband_deg_;
        int black_color_; // For LED control

        // Frame counter for periodic motor status checks
        int frame_counter_; // **Moved to match initialization order**

        // Current detected label and tracked object center (for getters)
        std::string current_detected_label_;
        Robot::Point2D current_tracked_object_center_;
        int detection_score_;
        double camera_focal_length_px_;
        std::map<std::string, double> known_object_real_heights_m_;
        double current_object_distance_m_;

        std::chrono::steady_clock::time_point last_motor_command_time_;
        int motor_command_interval_ms_; // Interval for sending motor commands (in milliseconds)

        // --- Private Helper Methods (implementing Head.cpp functionality) ---

        // Initializes the Unix Domain Socket server and waits for connection
        int InitializeSocketServer();
        // Initializes the MJPG Streamer
        bool InitializeStreamer();
        // Handles sending frame data over the socket
        bool SendFrameData(Robot::Image *frame);
        // Handles receiving detection results over the socket and parsing them
        std::vector<ParsedDetection> ReceiveDetectionResults();
        // Function to parse the detection string received from Python
        std::vector<ParsedDetection> ParseDetectionOutput(const std::string &output);
        // Basic function to draw bounding boxes on the RGB image
        void DrawBoundingBox(Robot::Image *image, const ParsedDetection &detection);
        // Handles head tracking logic based on detection results
        void UpdateHeadTracking(const std::vector<ParsedDetection> &detections);
        // Helper to receive exact number of bytes from socket
        std::string ReceiveExact(int sock_fd, size_t num_bytes);

         void LoadDistanceEstimationSettings(minIni* ini);

        // New methods to replace Head.cpp functionality:
        void LoadHeadSettings(minIni *ini);              // Load head-specific settings from INI
        void CheckLimit();                               // Apply angle limits
        void MoveToHome();                               // Move head to home position
        void MoveByAngle(double pan, double tilt);       // Set target angles
        void MoveByAngleOffset(double pan, double tilt); // Move by angle offset
        void InitTracking();                             // Reset tracking errors
        void UpdateHeadAngles(Robot::Point2D err);       // Calculate new angles based on error (replaces MoveTracking(Point2D err))
        void ApplyHeadAngles();                          // Apply calculated angles to motors (replaces Head::Process())
        double Value2Deg(int value);
        int Deg2Value(double angle);
        void SetMotorPIDAndSpeed();
    };
}

#endif // HEADTRACKING_H_
