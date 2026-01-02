/*
 * main.cpp
 *
 * Modified to use initialization routine from react_on_object_detect/main.cpp
 * while preserving gesture detection logic.
 */

#include "ConsoleColors.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <iostream>  // For std::cout, std::cerr, std::endl
#include <cstdlib>   // Required for system()
#include <pthread.h> // Required for threading
#include <string>    // Required for std::string
#include <chrono>    // Required for timing
#include <thread>    // Required for std::this_thread::sleep_for
#include <cctype>
#include "algorithm"
#include <signal.h>

#include "minIni.h" // For INI file loading
#include "HeadTracking.h"
#include "LeftArmController.h"
#include "RightArmController.h"
#include "LegsController.h"
#include "LinuxDARwIn.h" // Include for Motion Framework components (MotionManager, Action)
#include "LinuxActionScript.h"

// --- Configuration ---
#define INI_FILE_PATH "../../../../Data/config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0" // Verify this path is correct!
#define MOTION_FILE_PATH "../../../../Data/motion_4096.bin"

// Define action page numbers for gesture demo
#define ACTION_PAGE_WAVE 15
#define ACTION_PAGE_READY 1
#define DETECT_THRESHOLD 1

using namespace Robot;

// Helper function to change directory to the executable's path
void change_current_dir()
{
    char exepath[1024] = {0};
    if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

// Wrapper to run an action page safely
void run_action(int action_page)
{
    // Walking module must be removed before running Action to prevent conflict
    MotionManager::GetInstance()->RemoveModule(static_cast<MotionModule *>(Walking::GetInstance()));
    MotionManager::GetInstance()->SetEnable(true); // Must be active to run Action

    // Disable head joints in MotionManager so HeadTracking can control them if needed,
    // or simply to prevent fighting during the action if the action doesn't use them.
    MotionManager::GetInstance()->SetJointEnableState(JointData::ID_HEAD_PAN, false);
    MotionManager::GetInstance()->SetJointEnableState(JointData::ID_HEAD_TILT, false);

    Action::GetInstance()->Start(action_page);

    while (Action::GetInstance()->IsRunning())
        usleep(8 * 1000);

    // Re-add walking module (good practice to restore state, even if not walking immediately)
    MotionManager::GetInstance()->AddModule(static_cast<MotionModule *>(Walking::GetInstance()));
    MotionManager::GetInstance()->SetEnable(false);
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
    printf("\n===== Darwin-OP Gesture Mode (Initialized) =====\n\n");

    signal(SIGPIPE, SIG_IGN);
    change_current_dir(); // Change current directory to executable's location

    std::cout << "Cleaning up stale processes..." << std::endl;
    system("pkill -9 -f gesture_detector.py"); // Ensure the specific gesture script is killed
    system("pkill -9 mjpg_streamer");

    // Load INI settings
    minIni *ini = new minIni(INI_FILE_PATH);
    if (!ini)
    {
        std::cerr << "ERROR: Failed to load INI file." << std::endl;
        return -1;
    }

    Robot::Action::GetInstance()->LoadFile(MOTION_FILE_PATH); // Load motion file for Action module

    // --- Camera Initialization ---
    std::cout << "INFO: Initializing camera..." << std::endl;
    LinuxCamera::GetInstance()->Initialize(0); // Initialize with device index 0
    LinuxCamera::GetInstance()->LoadINISettings(ini);
    std::cout << "INFO: Camera initialized and settings loaded." << std::endl;

    // --- Initialize Motion Framework Components ---
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);

    // Get MotionManager and Action singletons
    MotionManager *motion_manager = MotionManager::GetInstance();
    Action *action_module = Action::GetInstance();

    if (motion_manager->Initialize(&cm730) == false)
    {
        std::cerr << "ERROR: Failed to initialize Motion Manager in main!" << std::endl;
        delete ini;
        return -1;
    }

    motion_manager->LoadINISettings(ini);
    motion_manager->AddModule((MotionModule *)action_module);

    // Start the motion timer
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(motion_manager);
    motion_timer->Start();

    // Initialize Controllers
    LeftArmController left_arm_controller(&cm730);
    RightArmController right_arm_controller(&cm730);
    LegsController legs_controller(&cm730);

    // --- Initialize Head Tracking ---
    HeadTracking *head_tracker = HeadTracking::GetInstance();

    // NOTE: passing '3' to Initialize generally signifies Gesture Mode/Script
    if (!head_tracker->Initialize(ini, &cm730, 3))
    {
        std::cerr << "ERROR: HeadTracking initialization failed. Exiting." << std::endl;
        motion_timer->Stop();
        MotionManager::GetInstance()->SetEnable(false);
        MotionManager::GetInstance()->RemoveModule((MotionModule *)action_module);
        delete ini;
        delete motion_timer;
        return -1;
    }

    // --- Initial Pose ---
    std::cout << "INFO: Playing initial Ready action (Page " << ACTION_PAGE_READY << ")..." << std::endl;
    run_action(ACTION_PAGE_READY);

    // --- Start Vision Thread ---
    pthread_t tracking_thread;
    std::cout << "INFO: Creating HeadTracking thread..." << std::endl;
    int thread_create_status = pthread_create(&tracking_thread, NULL, HeadTrackingThread, head_tracker);

    if (thread_create_status != 0)
    {
        std::cerr << "ERROR: Failed to create HeadTracking thread: " << strerror(thread_create_status) << std::endl;
        head_tracker->Cleanup();
        motion_timer->Stop();
        MotionManager::GetInstance()->SetEnable(false);
        MotionManager::GetInstance()->RemoveModule((MotionModule *)action_module);
        delete ini;
        delete motion_timer;
        return -1;
    }
    std::cout << "INFO: HeadTracking thread created successfully." << std::endl;

    std::cout << "\033[1;32mREADY: Waiting for hand_wave...\033[0m" << std::endl;

    // --- Main Logic Loop ---
    int wave_counter = 0;
    while (true)
    {
        std::string label = head_tracker->GetDetectedLabel();

        if (!label.empty())
        {
            std::cout << "[CPP Main] Saw Label: " << label << std::endl;
        }

        if (label == "hand_wave")
        {
            wave_counter++;
            if (wave_counter >= DETECT_THRESHOLD)
            {
                std::cout << "\n\033[1;35m>>> WAVE CONFIRMED! Executing Action... \033[0m" << std::endl;
                run_action(ACTION_PAGE_WAVE);
                wave_counter = 0;

                std::cout << "INFO: Back to Ready." << std::endl;
                run_action(ACTION_PAGE_READY);
                std::cout << "\033[1;32mREADY: Waiting for hand_wave...\033[0m" << std::endl;
            }
        }
        else
        {
            if (wave_counter > 0)
                wave_counter--;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // (Unreachable in this infinite loop, but good for structure)
    std::cout << "INFO: Main loop terminated." << std::endl;
    pthread_join(tracking_thread, NULL);
    motion_timer->Stop();
    delete ini;
    delete motion_timer;

    return 0;
}