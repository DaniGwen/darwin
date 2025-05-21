/*
 * HeadTracking.h
 *
 * Created on: May 17, 2025
 * Author: gwenski
 * Description: Singleton class to manage head tracking, socket communication
 *                     with a detector script, and MJPG streaming for a DARwIn-OP robot.
 *                     Receives initialized Motion Framework (MotionManager, Head) pointers.
 */

#ifndef HEADTRACKING_H_
#define HEADTRACKING_H_

#include <string>   // For std::string
#include <vector>   // For std::vector
#include <memory>   // For std::unique_ptr
#include <iostream> // For std::cerr, std::cout

// Headers for Unix Domain Sockets
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h> // For errno and strerror

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h" // Assuming this provides basic robot control structures (MotionManager, Head, Point2D, JointData)
#include "minIni.h"      // For INI file loading
namespace Robot
{
      class MotionManager; // Forward declaration if still needed for other reasons, but not for HeadTracking's direct control
      class Head;          // Forward declaration for Head
      class Point2D;       // Forward declaration for Point2D
}

class HeadTracking
{
private:
      int client_socket_;
      mjpg_streamer *streamer_;
      minIni *ini_settings_;
      Robot::Head *head_module_; // Pointer to the Head singleton
      CM730 *cm730_;
      Image *rgb_display_frame_;
      int no_target_count_;
      double pan_error_scale_;
      double tilt_error_scale_;
      double pan_deadband_deg_;
      double tilt_deadband_deg_;
      int black_color_;
      std::string current_detected_label_;
      Robot::Point2D current_tracked_object_center_;

      // Private helper methods
      int InitializeSocketServer();
      bool InitializeStreamer();
      bool SendFrameData(Image *frame);
      std::vector<ParsedDetection> ReceiveDetectionResults();
      std::vector<ParsedDetection> ParseDetectionOutput(const std::string &output);
      void DrawBoundingBox(Image *image, const ParsedDetection &detection);
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
