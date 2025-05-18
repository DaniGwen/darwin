/*
 * main.cpp
 *
 * Created on: 2011. 1. 4.
 * Author: robotis
 * Modified for Edge TPU Object Detection via Unix Domain Socket
 * Refactored into HeadTracking singleton class.
 * Added automatic startup of the Python detector script.
 * Motion Framework initialization AND CLEANUP moved to main.
 * Fixed multiple definition error for SOCKET_PATH.
 * Added explicit motion framework shutdown steps.
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <iostream>
#include <cstdlib> // Required for system()

#include "minIni.h"       // For INI file loading
#include "HeadTracking.h" // Include the new HeadTracking class header (declares SOCKET_PATH)
#include "LinuxDARwIn.h" // Include for Motion Framework components

// --- Python Script Configuration ---
// IMPORTANT: Set the correct path to your Python detector script
const char *PYTHON_SCRIPT_PATH = "/home/darwin/darwin/aiy-maker-kit/examples/custom_detect_objects.py"; // Corrected path

#define INI_FILE_PATH          "config.ini"
#define U2D_DEV_NAME            "/dev/ttyUSB0" // Verify this path is correct!

void change_current_dir()
{
      char exepath[1024] = {0};
      if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
            chdir(dirname(exepath));
}

int main(void)
{
      printf("\n===== Head tracking with Object Detection via Unix Domain Socket =====\n\n");

      change_current_dir();

      // Load INI settings
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


      // --- Initialize Motion Framework Components (in main) ---
      // These are instantiated and initialized here before HeadTracking uses them.
      LinuxCM730 linux_cm730(U2D_DEV_NAME);
      CM730 cm730(&linux_cm730);

      // Get MotionManager singleton and initialize it with the CM730 instance
      MotionManager* motion_manager = MotionManager::GetInstance();
      Head* head_module = Head::GetInstance(); // Get Head singleton now as well

      if (motion_manager->Initialize(&cm730) == false)
      {
            std::cerr << "ERROR: Failed to initialize Motion Manager in main!" << std::endl;
            delete ini;
            return -1;
      }

      // Load MotionManager settings from INI
      motion_manager->LoadINISettings(ini);

      // Start the Motion Timer (often needed for MotionManager to function)
      // Assuming LinuxMotionTimer is not a singleton and needs to be managed here.
      // If it's a singleton, get instance and start it.
      LinuxMotionTimer *motion_timer = new LinuxMotionTimer(motion_manager);
      motion_timer->Start();


      // --- Initialize and Run HeadTracking ---
      HeadTracking* head_tracker = HeadTracking::GetInstance();

      // Pass the INI settings to HeadTracking for its specific configuration
      // HeadTracking will get the MotionManager and Head singletons internally.
      if (!head_tracker->Initialize(ini)) {
            std::cerr << "ERROR: HeadTracking initialization failed. Exiting." << std::endl;
            // Perform motion framework cleanup before exiting
            motion_timer->Stop();
            motion_manager->SetEnable(false);
            motion_manager->RemoveModule((MotionModule *)head_module);
            delete ini;
            delete motion_timer;
            return -1;
      }

      // Run the main tracking loop (this will block)
      head_tracker->Run();

      // --- Cleanup Resources ---
      // First, cleanup HeadTracking specific resources (socket, streamer, display frame)
      head_tracker->Cleanup();

      // Then, perform explicit motion framework shutdown in reverse order of initialization
      std::cout << "INFO: Shutting down motion framework..." << std::endl;
      motion_timer->Stop();
      motion_manager->SetEnable(false); // Disable motion
      motion_manager->RemoveModule((MotionModule *)head_module); // Remove Head module

      // LinuxCM730 and CM730 are stack allocated and will be cleaned up when main exits.
      // Their destructors should handle closing the serial port.

      // Finally, cleanup other dynamically allocated objects
      delete ini; // Clean up ini
      delete motion_timer; // Clean up timer

      std::cout << "INFO: Main program exiting." << std::endl;

      return 0;
}
