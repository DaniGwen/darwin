/*
 * main.cpp
 *
 * Created on: 2025. 5. 19.
 * Author: gwenski
 * Modified for Edge TPU Object Detection via Unix Domain Socket
 * Refactored into HeadTracking singleton class.
 * Python script startup moved to HeadTracking::Initialize.
 * Motion Framework initialization and cleanup in main.
 * Fixed multiple definition error for SOCKET_PATH.
 * Passes initialized MotionManager and Head pointers to HeadTracking.
 * Head tracking logic now runs in a separate thread.
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <iostream>
#include <cstdlib>  // Required for system()
#include <pthread.h> // Required for threading

#include "minIni.h"       // For INI file loading
#include "HeadTracking.h" // Include the new HeadTracking class header (declares SOCKET_PATH)
#include "LinuxDARwIn.h"  // Include for Motion Framework components

// --- Python Script Configuration ---
// PYTHON_SCRIPT_PATH is now defined in HeadTracking.cpp

#define INI_FILE_PATH "../../../../Data/config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0" // Verify this path is correct!

void change_current_dir()
{
    char exepath[1024] = {0};
    if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

// Thread entry point function for HeadTracking
void* HeadTrackingThread(void* arg)
{
    // Cast the argument back to a HeadTracking pointer
    HeadTracking* head_tracker = static_cast<HeadTracking*>(arg);

    if (head_tracker) {
        // Run the main tracking loop in this thread
        head_tracker->Run();
    } else {
        std::cerr << "ERROR: HeadTrackingThread received a null pointer." << std::endl;
    }

    // Threads should return NULL or a specific value upon completion
    return NULL;
}


int main(void)
{
    printf("\n===== Head tracking with Object Detection via Unix Domain Socket (Multithreaded) =====\n\n");

    change_current_dir();

    // Load INI settings
    minIni *ini = new minIni(INI_FILE_PATH);
    if (!ini)
    {
        std::cerr << "ERROR: Failed to load INI file." << std::endl;
        return -1;
    }

    // --- Camera Initialization (kept in main) ---
    // It's often good practice to initialize the camera early in main
    // as it's a fundamental system resource.
    std::cout << "INFO: Initializing camera..." << std::endl;
    LinuxCamera::GetInstance()->Initialize(0); // Initialize with device index 0
    LinuxCamera::GetInstance()->LoadINISettings(ini);
    std::cout << "INFO: Camera initialized and settings loaded." << std::endl;


    // --- Initialize Motion Framework Components (in main) ---
    // These are instantiated and initialized here before HeadTracking uses them.
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);

    // Get MotionManager singleton and initialize it with the CM730 instance
    Robot::MotionManager *motion_manager = Robot::MotionManager::GetInstance();

    if (motion_manager->Initialize(&cm730) == false)
    {
        std::cerr << "ERROR: Failed to initialize Motion Manager in main!" << std::endl;
        delete ini;
        return -1;
    }

    // Load MotionManager settings from INI
    motion_manager->LoadINISettings(ini);

    // Start the Motion Timer (often needed for MotionManager to function)
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(motion_manager);
    motion_timer->Start();

    // --- Initialize HeadTracking ---
    HeadTracking *head_tracker = HeadTracking::GetInstance();

    // Pass the INI settings and the initialized motion framework singletons to HeadTracking
    // The Python script startup is now handled inside head_tracker->Initialize()
    if (!head_tracker->Initialize(ini, &cm730))
    {
        std::cerr << "ERROR: HeadTracking initialization failed. Exiting." << std::endl;
        // Perform motion framework cleanup before exiting
        motion_timer->Stop();
        motion_manager->SetEnable(false);
        delete ini;
        delete motion_timer;
        return -1;
    }
    
    // --- Create and Start HeadTracking Thread ---
    pthread_t tracking_thread;
    std::cout << "INFO: Creating HeadTracking thread..." << std::endl;
    int thread_create_status = pthread_create(&tracking_thread, NULL, HeadTrackingThread, head_tracker);

    if (thread_create_status != 0) {
        std::cerr << "ERROR: Failed to create HeadTracking thread: " << strerror(thread_create_status) << std::endl;
        // Cleanup initialized resources before exiting
        head_tracker->Cleanup(); // Cleanup HeadTracking resources
        motion_timer->Stop();
        motion_manager->SetEnable(false);
        delete ini;
        delete motion_timer;
        return -1;
    }
    std::cout << "INFO: HeadTracking thread created successfully." << std::endl;


    // --- Main Loop (for other tasks) ---
    // The main thread can now perform other tasks here.
    // For demonstration, we'll just keep the main thread alive.
    std::cout << "INFO: Main thread running, HeadTracking in separate thread. Press Ctrl+C to exit." << std::endl;

    // Wait for the tracking thread to finish (e.g., on socket error)
    // In a real robot application, main might have its own loop
    // managing other modules or waiting for a shutdown signal.
    pthread_join(tracking_thread, NULL);

    std::cout << "INFO: HeadTracking thread finished. Proceeding with main cleanup." << std::endl;

    // --- Cleanup Resources ---
    // First, cleanup HeadTracking specific resources (socket, streamer, display frame)
    // This is also handled by the thread's exit or explicit cleanup call if the thread exits gracefully.
    // head_tracker->Cleanup(); // Can be called here too for certainty, but thread exit should handle it.

    // Then, perform explicit motion framework shutdown in reverse order of initialization
    std::cout << "INFO: Shutting down motion framework..." << std::endl;
    motion_timer->Stop();
    motion_manager->SetEnable(false);                          // Disable motion

    // LinuxCM730 and CM730 are stack allocated and will be cleaned up when main exits.
    // Their destructors should handle closing the serial port.

    // Finally, cleanup other dynamically allocated objects
    delete ini;          // Clean up ini
    delete motion_timer; // Clean up timer

    std::cout << "INFO: Main program exiting." << std::endl;

    return 0;
}
