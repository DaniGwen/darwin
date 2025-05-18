/*
 * HeadTracking.h
 *
 * Created on: May 17, 2025
 * Author: Your Name
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

// Structure to hold parsed detection data received from Python
struct ParsedDetection
{
      std::string label;
      float score;
      float xmin, ymin, xmax, ymax; // Normalized coordinates [0.0, 1.0]
};

// Declare socket path using extern - the definition is in HeadTracking.cpp
extern const char *SOCKET_PATH;

class HeadTracking
{
public:
      // Singleton access method
      static HeadTracking *GetInstance();

      // Initialization method
      // Requires the minIni instance for loading settings and
      // initialized MotionManager and Head singleton pointers.
      // Returns true on success, false on failure.
      bool Initialize(minIni *ini, Robot::MotionManager *motion_manager, Robot::Head *head_module, CM730 *cm730);

      // Main tracking loop
      // This function will run the main processing loop (capture, send, receive, track).
      // It will block until an error occurs or the loop is exited internally.
      void Run();

      // Cleanup method
      void Cleanup();

      // Destructor (private to enforce singleton)
      ~HeadTracking();

private:
      // Private constructor to enforce singleton pattern
      HeadTracking();

      // Delete copy constructor and assignment operator
      HeadTracking(const HeadTracking &) = delete;
      HeadTracking &operator=(const HeadTracking &) = delete;

      // --- Member Variables ---
      int client_socket_;       // File descriptor for the client socket connection
      mjpg_streamer *streamer_; // Pointer to the MJPG streamer instance
      minIni *ini_settings_;    // Pointer to loaded INI settings (owned by main)

      // Pointers to DARwIn-OP framework singletons (passed in Initialize, not owned)
      Robot::MotionManager *motion_manager_;
      Robot::Head *head_module_;
      // LinuxMotionTimer is assumed to be managed by MotionManager or main.

      // Image buffer for the output frame with detections drawn on it
      Image *rgb_display_frame_;

      // Tracking state variables
      int no_target_count_;
      const int NO_TARGET_MAX_COUNT = 30; // Number of frames to wait before initiating scan

      // Tuning Parameters for Centering (loaded from INI in Initialize)
      double pan_error_scale_;
      double tilt_error_scale_;
      double pan_deadband_deg_;
      double tilt_deadband_deg_;

      // Eye color values for LED panel
      int magenta_color_;
      int cyan_color_;
      int white_color_;
      int yellow_color_;
      int black_color_;

      // --- Private Helper Methods ---

      // Initializes the Unix Domain Socket server and waits for connection
      // Returns client_sock file descriptor on success, -1 on failure
      int InitializeSocketServer();

      // Initializes the Camera
      // Returns true on success, false on failure
      bool InitializeCamera();

      // Initializes the MJPG Streamer
      // Returns true on success, false on failure
      bool InitializeStreamer();

      // Handles sending frame data over the socket
      // Returns true on success, false on failure (e.g., connection closed)
      bool SendFrameData(Image *frame);

      // Handles receiving detection results over the socket and parsing them
      // Returns vector of detections on success, empty vector on failure or no detections.
      // Returns empty vector AND prints error on socket read failure (e.g., connection closed).
      std::vector<ParsedDetection> ReceiveDetectionResults();

      // Function to parse the detection string received from Python
      std::vector<ParsedDetection> ParseDetectionOutput(const std::string &output);

      // Basic function to draw bounding boxes on the RGB image
      void DrawBoundingBox(Image *image, const ParsedDetection &detection);

      // Handles head tracking logic based on detection results
      // Uses the motion_manager_ and head_module_ member pointers.
      void UpdateHeadTracking(const std::vector<ParsedDetection> &detections);

      // Helper to receive exact number of bytes from socket
      // Returns received data or empty string on error/disconnect
      std::string ReceiveExact(int sock_fd, size_t num_bytes);
};

#endif /* HEADTRACKING_H_ */
