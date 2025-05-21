#ifndef HEADTRACKING_H_
#define HEADTRACKING_H_

#include <string>
#include <vector>

#include "Point.h" // Make sure Point2D is defined (usually in Robot::Point)
#include "Head.h"  // Include Head.h as HeadTracking will now directly use Head
#include "CM730.h" // Include CM730.h for CM730 class

// Forward declarations for Robot namespace classes if needed,
// though including their headers is generally safer.
namespace Robot
{
    class MotionManager; // Still forward declare if HeadTracking.h is included by files that use MotionManager
    class Head;
    class Point2D;
    class Image; // Assuming Image class is in Robot namespace or globally accessible
}

// Structure to hold parsed detection data received from Python
// Moved here so it's accessible to HeadTracking.h and HeadTracking.cpp
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
    mjpg_streamer *streamer_;
    minIni *ini_settings_;
    Robot::Head *head_module_; // Pointer to the Head singleton
    CM730 *cm730_;
    Robot::Image *rgb_display_frame_; // Use Robot::Image if it's in that namespace
    int no_target_count_;
    double pan_error_scale_;
    double tilt_error_scale_;
    double pan_deadband_deg_;
    double tilt_deadband_deg_;
    int black_color_;
    std::string current_detected_label_;
    Robot::Point2D current_tracked_object_center_;

    // Define NO_TARGET_MAX_COUNT here
    static const int NO_TARGET_MAX_COUNT = 30; // Number of frames to wait before initiating scan

    // Private helper methods
    int InitializeSocketServer();
    bool InitializeStreamer();
    bool SendFrameData(Robot::Image *frame); // Use Robot::Image
    std::vector<ParsedDetection> ReceiveDetectionResults();
    std::vector<ParsedDetection> ParseDetectionOutput(const std::string &output);
    void DrawBoundingBox(Robot::Image *image, const ParsedDetection &detection); // Use Robot::Image
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
