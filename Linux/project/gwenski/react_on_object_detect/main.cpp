/*
 * main.cpp
 *
 * Created on: 2025. 5. 19.
 * Author: gwenski
 * Description: Main program for head tracking with object detection,
 * triggering robot actions based on detected object labels.
 * Runs HeadTracking in a separate thread, with Head control
 * managed directly by HeadTracking, not MotionManager.
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
#define INI_FILE_PATH "../../../../Data/config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0" // Verify this path is correct!
#define MOTION_FILE_PATH "../../../../Data/motion_4096.bin"

// Define action page numbers for different detected objects
#define ACTION_PAGE_WAVE        7
#define ACTION_PAGE_DOG         11
#define ACTION_PAGE_CAT         12
#define ACTION_PAGE_SPORTS_BALL 13
#define ACTION_PAGE_BOTTLE      14
#define ACTION_PAGE_STAND       1 // Example standby/initial pose action

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

    return NULL;
}

int main(void)
{
    printf("\n===== Head tracking with Object Detection via Unix Domain Socket (Multithreaded) =====\n\n");

    change_current_dir(); // Change current directory to executable's location

    // Load INI settings
    minIni *ini = new minIni(INI_FILE_PATH);
    if (!ini)
    {
        std::cerr << "ERROR: Failed to load INI file." << std::endl;
        return -1;
    }

    Action::GetInstance()->LoadFile(MOTION_FILE_PATH); // Load motion file for Action module

    // --- Camera Initialization ---
    std::cout << "INFO: Initializing camera..." << std::endl;
    LinuxCamera::GetInstance()->Initialize(0); // Initialize with device index 0
    LinuxCamera::GetInstance()->LoadINISettings(ini);
    std::cout << "INFO: Camera initialized and settings loaded." << std::endl;

    // --- Initialize Motion Framework Components ---
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);

    // Get MotionManager, Head, and Action singletons
    Robot::MotionManager *motion_manager = Robot::MotionManager::GetInstance();
    Robot::Head *head_module = Robot::Head::GetInstance();       // Get Head singleton
    Robot::Action *action_module = Robot::Action::GetInstance(); // Get Action singleton

    // Initialize MotionManager
    if (motion_manager->Initialize(&cm730) == false)
    {
        std::cerr << "ERROR: Failed to initialize Motion Manager in main!" << std::endl;
        delete ini;
        return -1;
    }

    // Load MotionManager settings from INI
    motion_manager->LoadINISettings(ini);

    // Add Action module to MotionManager (Head is handled by HeadTracking directly)
    motion_manager->AddModule((MotionModule *)action_module);

    // Start the Motion Timer
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(motion_manager);
    motion_timer->Start();

    // Enable MotionManager
    MotionManager::GetInstance()->SetEnable(true);

    // Load Head's general settings (limits, home position) from INI
    // HeadTracking will handle enabling torque and setting P-gains.
    head_module->LoadINISettings(ini);

    // --- Initialize HeadTracking ---
    HeadTracking *head_tracker = HeadTracking::GetInstance();

    // Pass the INI settings, Head module, and CM730 instance to HeadTracking
    // HeadTracking will now manage the Head's motor control directly.
    if (!head_tracker->Initialize(ini, head_module, &cm730))
    {
        std::cerr << "ERROR: HeadTracking initialization failed. Exiting." << std::endl;
        // Perform motion framework cleanup before exiting
        motion_timer->Stop();
        MotionManager::GetInstance()->SetEnable(false);
        MotionManager::GetInstance()->RemoveModule((MotionModule *)action_module); // Remove Action module
        delete ini;
        delete motion_timer;
        return -1;
    }

    // Play initial standby action
    std::cout << "INFO: Playing initial standby action (Page " << ACTION_PAGE_STAND << ")..." << std::endl;
    action_module->Start(ACTION_PAGE_STAND);
    // Wait for the action to complete before proceeding
    while (action_module->IsRunning())
        usleep(8 * 1000); // Small delay to prevent busy-waiting

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
        MotionManager::GetInstance()->SetEnable(false);
        MotionManager::GetInstance()->RemoveModule((MotionModule *)action_module); // Remove Action module
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
        bool is_action_playing = action_module->IsRunning();

        // --- Action Triggering Logic ---
        // Only start a new action if no action is currently playing AND the detected label is new
        if (!is_action_playing)
        {
            if (detected_object_label == "person" && current_action_label != "person")
            {
                std::cout << "INFO: Detected person. Playing action (Page " << ACTION_PAGE_WAVE << ")..." << std::endl;
                action_module->Start(ACTION_PAGE_WAVE);
                // Wait for action to complete before allowing new actions
                while (action_module->IsRunning())
                    usleep(8 * 1000);
                current_action_label = "person";
            }
            else if (detected_object_label == "dog" && current_action_label != "dog")
            {
                std::cout << "INFO: Detected dog. Playing action (Page " << ACTION_PAGE_DOG << ")..." << std::endl;
                action_module->Start(ACTION_PAGE_DOG);
                while (action_module->IsRunning())
                    usleep(8 * 1000);
                current_action_label = "dog";
            }
            else if (detected_object_label == "cat" && current_action_label != "cat")
            {
                std::cout << "INFO: Detected cat. Playing action (Page " << ACTION_PAGE_CAT << ")..." << std::endl;
                action_module->Start(ACTION_PAGE_CAT);
                while (action_module->IsRunning())
                    usleep(8 * 1000);
                current_action_label = "cat";
            }
            else if (detected_object_label == "sports ball" && current_action_label != "sports ball")
            {
                std::cout << "INFO: Detected sports ball. Playing action (Page " << ACTION_PAGE_SPORTS_BALL << ")..." << std::endl;
                action_module->Start(ACTION_PAGE_SPORTS_BALL);
                while (action_module->IsRunning())
                    usleep(8 * 1000);
                current_action_label = "sports ball";
            }
            else if (detected_object_label == "bottle" && current_action_label != "bottle")
            {
                std::cout << "INFO: Detected bottle. Playing action (Page " << ACTION_PAGE_BOTTLE << ")..." << std::endl;
                action_module->Start(ACTION_PAGE_BOTTLE);
                while (action_module->IsRunning())
                    usleep(8 * 1000);
                current_action_label = "bottle";
            }
            else if (detected_object_label == "none" && current_action_label != "standby")
            {
                // If no specific object is detected and we are not already in standby, go to standby
                std::cout << "INFO: No target detected. Returning to standby action (Page " << ACTION_PAGE_STAND << ")..." << std::endl;
                action_module->Start(ACTION_PAGE_STAND);
                while (action_module->IsRunning())
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
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Check every 100ms
    }

    // --- Cleanup Resources ---
    std::cout << "INFO: Main loop terminated. Waiting for HeadTracking thread to join..." << std::endl;
    pthread_join(tracking_thread, NULL);
    std::cout << "INFO: HeadTracking thread joined." << std::endl;

    // Perform explicit motion framework shutdown
    std::cout << "INFO: Shutting down motion framework..." << std::endl;
    motion_timer->Stop();
    MotionManager::GetInstance()->SetEnable(false);
    // Only remove Action module, Head is not a module of MotionManager anymore
    MotionManager::GetInstance()->RemoveModule((MotionModule *)action_module);

    delete ini;
    delete motion_timer;

    std::cout << "INFO: Main program exiting." << std::endl;

    return 0;
}
