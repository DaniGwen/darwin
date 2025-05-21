#ifndef HEADTRACKING_H_
#define HEADTRACKING_H_

#include <string>   // For std::string
#include <vector>   // For std::vector

// Include necessary headers for types used in HeadTracking class members and methods
#include "Point.h"        // For Robot::Point2D
#include "Head.h"         // For Robot::Head and JointData (often pulled in by Head.h)
#include "CM730.h"        // For CM730 class
#include "Camera.h"       // For Camera::WIDTH, Camera::HEIGHT, and Robot::Image
#include "mjpg_streamer.h" // For mjpg_streamer class
#include "minIni.h"       // For minIni class

// Forward declarations for Robot namespace classes if they are only used as pointers/references
// and their full definition is not needed in the header.
// However, in this case, we need full definitions for member variables and method parameters,
// so including the respective headers is more robust.
namespace Robot
{
    class MotionManager; // Still forward declare if HeadTracking.h is included by files that use MotionManager
    // Robot::Head, Robot::Point2D, Robot::Image are now fully defined by includes above.
}

// Structure to hold parsed detection data received from Python
// This must be defined BEFORE the HeadTracking class if HeadTracking uses it.
struct ParsedDetection
{
    std::string label;
    float score;
    float xmin, ymin, xmax, ymax; // Normalized coordinates [0.0, 1.0]
};

class HeadTracking
{
private:
    int client_socket_;
    mjpg_streamer *streamer_; // Full definition of mjpg_streamer is now available
    minIni *ini_settings_;
    Robot::Head *head_module_; // Pointer to the Head singleton
    CM730 *cm730_;             // Full definition of CM730 is now available
    Robot::Image *rgb_display_frame_; // Full definition of Robot::Image is now available
    int no_target_count_;
    double pan_error_scale_;
    double tilt_error_scale_;
    double pan_deadband_deg_;
    double tilt_deadband_deg_;
    int black_color_;
    std::string current_detected_label_;
    Robot::Point2D current_tracked_object_center_;

    // Define NO_TARGET_MAX_COUNT here, as it's a static const member
    static const int NO_TARGET_MAX_COUNT = 30; // Number of frames to wait before initiating scan

    // Private helper methods
    int InitializeSocketServer();
    bool InitializeStreamer();
    bool SendFrameData(Robot::Image *frame);
    std::vector<ParsedDetection> ReceiveDetectionResults();
    std::vector<ParsedDetection> ParseDetectionOutput(const std::string &output);
    void DrawBoundingBox(Robot::Image *image, const ParsedDetection &detection);
    void UpdateHeadTracking(const std::vector<ParsedDetection> &detections);
    std::string ReceiveExact(int sock_fd, size_t num_bytes);

    // Private constructor for singleton pattern
    HeadTracking();
    // Prevent copy and assignment
    HeadTracking(const HeadTracking &) = delete;
    HeadTracking &operator=(const HeadTracking &) = delete;

public:
    // Public method to get the singleton instance
    static HeadTracking *GetInstance();

    // Modified Initialize signature to accept Head* and CM730*
    bool Initialize(minIni *ini, Robot::Head *head_module, CM730 *cm730);

    // Main tracking loop
    void Run();

    // Cleanup method
    void Cleanup();

    // Public getters for detected label and object center
    std::string GetDetectedLabel();
    Robot::Point2D GetTrackedObjectCenter();
};

#endif // HEADTRACKING_H_
