/*
 * main.cpp
 *
 * Created on: 2025. 5. 19.
 * Author: gwenski
 * Description: Main program for head tracking with object detection,
 *              triggering robot actions based on detected object labels.
 *              Runs HeadTracking in a separate thread.
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <iostream>
#include <cstdlib>   // Required for system()
#include <pthread.h> // Required for threading
#include <string>    // Required for std::string
#include <chrono>    // Required for timing (optional, for loop delay)
#include <thread>    // Required for std::this_thread::sleep_for (optional)

#include "minIni.h"       // For INI file loading
#include "HeadTracking.h" // Include the HeadTracking class header
#include "LinuxDARwIn.h"  // Include for Motion Framework components (MotionManager, Head, Action)

// --- Configuration ---
#define INI_FILE_PATH  "../../../../Data/config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0" // Verify this path is correct!
#define MOTION_FILE_PATH "../../../../Data/motion_4096.bin"

// Define action page numbers for different detected objects
#define ACTION_PAGE_WAVE "wave"
#define ACTION_PAGE_DOG 11
#define ACTION_PAGE_CAT 12
#define ACTION_PAGE_SPORTS_BALL 13
#define ACTION_PAGE_BOTTLE 14
#define ACTION_PAGE_STAND "stand"

// --- Global Flags/Variables (Consider using a shared state class if more complex) ---
// bool g_is_action_playing = false; // Flag to indicate if an action is currently playing

void change_current_dir()
{
    char exepath[1024] = {0};
    if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

// Thread entry point function for HeadTracking
void *HeadTrackingThread(void *arg)
{
    // Cast the argument back to a HeadTracking pointer
    HeadTracking *head_tracker = static_cast<HeadTracking *>(arg);

    if (head_tracker)
    {
        // Run the main tracking loop in this thread
        head_tracker->Run();
    }
    else
    {
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

    Action::GetInstance()->LoadFile(MOTION_FILE_PATH); // Load motion file for Action module

    // --- Camera Initialization (kept in main) ---
    std::cout << "INFO: Initializing camera..." << std::endl;
    LinuxCamera::GetInstance()->Initialize(0); // Initialize with device index 0
    LinuxCamera::GetInstance()->LoadINISettings(ini);
    std::cout << "INFO: Camera initialized and settings loaded." << std::endl;

    // --- Initialize Motion Framework Components (in main) ---
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);

    // Get MotionManager, Head, and Action singletons
    Robot::MotionManager *motion_manager = Robot::MotionManager::GetInstance();
    Robot::Head *head_module = Robot::Head::GetInstance();

    // Initialize MotionManager
    if (motion_manager->Initialize(&cm730) == false)
    {
        std::cerr << "ERROR: Failed to initialize Motion Manager in main!" << std::endl;
        delete ini;
        return -1;
    }

    // --- Initialize HeadTracking ---
    HeadTracking *head_tracker = HeadTracking::GetInstance();

    // Load MotionManager settings from INI
    motion_manager->LoadINISettings(ini);
    motion_manager->AddModule((MotionModule *)Action::GetInstance());
    motion_manager->SetEnable(true); // Enable MotionManager

    // Start the Motion Timer
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(motion_manager);
    motion_timer->Start();

    // Play initial standby action
    std::cout << "INFO: Playing initial standby action (Page " << ACTION_PAGE_STAND << ")..." << std::endl;
    Action::GetInstance()->Start(ACTION_PAGE_STAND);
        while (Action::GetInstance()->IsRunning())
            usleep(8 * 1000);

    // Pass the INI settings and the initialized motion framework singletons to HeadTracking
    if (!head_tracker->Initialize(ini, motion_manager, head_module, &cm730))
    {
        std::cerr << "ERROR: HeadTracking initialization failed. Exiting." << std::endl;
        // Perform motion framework cleanup before exiting
        motion_timer->Stop();
        motion_manager->SetEnable(false);
        motion_manager->RemoveModule((MotionModule *)head_module);
        motion_manager->RemoveModule((MotionModule *)Action::GetInstance()); // Remove Action module
        delete ini;
        delete motion_timer;
        return -1;
    }

    // --- Create and Start HeadTracking Thread ---
    pthread_t tracking_thread;
    std::cout << "INFO: Creating HeadTracking thread..." << std::endl;
    int thread_create_status = pthread_create(&tracking_thread, NULL, HeadTrackingThread, head_tracker);

    if (thread_create_status != 0)
    {
        std::cerr << "ERROR: Failed to create HeadTracking thread: " << strerror(thread_create_status) << std::endl;
        // Cleanup initialized resources before exiting
        head_tracker->Cleanup(); // Cleanup HeadTracking resources (socket, streamer, frame)
        motion_timer->Stop();
        motion_manager->SetEnable(false);
        motion_manager->RemoveModule((MotionModule *)head_module);
        motion_manager->RemoveModule((MotionModule *)Action::GetInstance()); // Remove Action module
        delete ini;
        delete motion_timer;
        return -1;
    }
    std::cout << "INFO: HeadTracking thread created successfully." << std::endl;

    // --- Main Loop (for checking labels and triggering actions) ---
    std::cout << "INFO: Main thread running, checking for detected objects to trigger actions. Press Ctrl+C to exit." << std::endl;

    std::string current_action_label = "standby"; // Keep track of the action currently playing

    while (1)
    {
        // Get the latest detected label from the HeadTracking thread
        std::string detected_object_label = head_tracker->GetDetectedLabel();

        // Check if an action is currently running
        bool is_action_playing = Action::GetInstance()->IsRunning();

        // --- Action Triggering Logic ---
        // Only start a new action if no action is currently playing
        if (!is_action_playing)
        {
            // Use if-else if to check the detected label
            if (detected_object_label == "person" && current_action_label != "person")
            {
                std::cout << "INFO: Detected person. Playing action (Page " << ACTION_PAGE_WAVE << ")..." << std::endl;
                Action::GetInstance()->Start(ACTION_PAGE_WAVE);
                while (Action::GetInstance()->IsRunning())
                    usleep(8 * 1000);
                current_action_label = "person";
            }
            else if (detected_object_label == "dog" && current_action_label != "dog")
            {
                // std::cout << "INFO: Detected dog. Playing action (Page " << ACTION_PAGE_DOG << ")..." << std::endl;
                // Action::GetInstance()->Start(ACTION_PAGE_DOG);
                // current_action_label = "dog";
            }
            else if (detected_object_label == "cat" && current_action_label != "cat")
            {
                // std::cout << "INFO: Detected cat. Playing action (Page " << ACTION_PAGE_CAT << ")..." << std::endl;
                // Action::GetInstance()->Start(ACTION_PAGE_CAT);
                // current_action_label = "cat";
            }
            else if (detected_object_label == "sports ball" && current_action_label != "sports ball")
            {
                // std::cout << "INFO: Detected sports ball. Playing action (Page " << ACTION_PAGE_SPORTS_BALL << ")..." << std::endl;
                // Action::GetInstance()->Start(ACTION_PAGE_SPORTS_BALL);
                // current_action_label = "sports ball";
            }
            else if (detected_object_label == "bottle" && current_action_label != "bottle")
            {
                // std::cout << "INFO: Detected bottle. Playing action (Page " << ACTION_PAGE_BOTTLE << ")..." << std::endl;
                // Action::GetInstance()->Start(ACTION_PAGE_BOTTLE);
                // current_action_label = "bottle";
            }
            else if (detected_object_label == "none" && current_action_label != "standby")
            {
                // If no specific object is detected and we are not already in standby, go to standby
                std::cout << "INFO: No target detected. Returning to standby action (Page " << ACTION_PAGE_STAND << ")..." << std::endl;
                Action::GetInstance()->Start(ACTION_PAGE_STAND);
                while (Action::GetInstance()->IsRunning())
                    usleep(8 * 1000);
                current_action_label = "standby";
            }
            // If the detected label is the same as the current action label,
            // we don't need to start the action again.
        }
        else
        {
            // An action is currently playing.
            // We could add logic here to potentially interrupt an action
            // if a higher priority object is detected, but for now we wait
            // for the current action to finish before starting a new one.
        }

        // Small delay in the main loop to avoid consuming too much CPU
        // Adjust this based on how frequently you need to check the label.
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Check every 100ms

        // You might add a mechanism to break this loop (e.g., a signal handler for Ctrl+C)
    }

    // --- Cleanup Resources ---
    // The HeadTracking thread will exit its Run() loop if the socket connection breaks.
    // We should wait for it to finish before cleaning up shared resources.
    std::cout << "INFO: Main loop terminated. Waiting for HeadTracking thread to join..." << std::endl;
    pthread_join(tracking_thread, NULL);
    std::cout << "INFO: HeadTracking thread joined." << std::endl;

    // Perform explicit motion framework shutdown in reverse order of initialization
    std::cout << "INFO: Shutting down motion framework..." << std::endl;
    motion_timer->Stop();
    motion_manager->SetEnable(false);
    // Disable motion
    motion_manager->RemoveModule((MotionModule *)head_module);           // Remove Head module
    motion_manager->RemoveModule((MotionModule *)Action::GetInstance()); // Remove Action module

    // Finally, cleanup other dynamically allocated objects
    delete ini;
    // Clean up ini
    delete motion_timer; // Clean up timer

    std::cout << "INFO: Main program exiting." << std::endl;

    return 0;
}
