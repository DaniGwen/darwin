/*
 * main.cpp
 *
 * Created on: 2025. 5. 19.
 * Author: gwenski
 * Description: Main program for head tracking with object detection,
 * triggering robot actions based on detected object labels.
 * Runs HeadTracking in a separate thread, with Head control
 * managed directly by HeadTracking, NOT MotionManager or Head.cpp.
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
#include <chrono>    // Required for timing (optional, for loop delay)
#include <thread>    // Required for std::this_thread::sleep_for (optional)
#include <cctype>
#include "algorithm"

#include "minIni.h" // For INI file loading
#include "HeadTracking.h"
#include "LeftArmController.h"
#include "RightArmController.h"
#include "LegsController.h"
#include "LinuxDARwIn.h" // Include for Motion Framework components (MotionManager, Action)

// --- Configuration ---
#define INI_FILE_PATH "../../../../Data/config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0" // Verify this path is correct!
#define MOTION_FILE_PATH "../../../../Data/motion_4096.bin"

// Define action page numbers for different detected objects
#define ACTION_PAGE_WAVE 7
#define ACTION_PAGE_DOG 11
#define ACTION_PAGE_CAT 12
#define ACTION_PAGE_SPORTS_BALL 13
#define ACTION_PAGE_BOTTLE 14
#define ACTION_PAGE_STAND 1 // Example standby/initial pose action

enum class BottleTaskState
{
    IDLE,              // Doing nothing, or searching
    WALKING_TO_BOTTLE, // Actively walking towards the bottle
    PICKING_UP,        // Close enough, performing the pickup motion
    DONE               // Task completed successfully
};

void change_current_dir()
{
    char exepath[1024] = {0};
    if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void run_action(int action_page)
{
    HeadTracking::SetTrackingEnabled(false);
    MotionManager::GetInstance()->SetEnable(true);

    Action::GetInstance()->Start(action_page);
    while (Action::GetInstance()->IsRunning())
        usleep(8 * 1000);

    MotionManager::GetInstance()->SetEnable(false);
    HeadTracking::SetTrackingEnabled(true);
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

// --- Action Handler Functions ---

void handlePersonDetected(LeftArmController &left_arm_controller,
                          std::string &current_action_label,
                          std::chrono::steady_clock::time_point &last_action_time,
                          int &person_detect_count_ref, // Pass by reference to reset
                          const std::chrono::steady_clock::time_point &current_time)
{
    std::cout << "INFO: Detected person consistently. Playing Wave" << std::endl;

    left_arm_controller.Wave();
    left_arm_controller.ToDefaultPose();

    current_action_label = "person";
    last_action_time = current_time;
    person_detect_count_ref = 0; // Reset counter
}

void handleBottleInteraction(BottleTaskState &state,
                             LegsController &legs_controller,
                             RightArmController &right_arm_controller,
                             HeadTracking *head_tracker,
                             std::string &current_action_label,
                             std::chrono::steady_clock::time_point &last_action_time,
                             const std::chrono::steady_clock::time_point &current_time)
{
    BallFollower follower = BallFollower();

    // IMPORTANT! Enable walking here because it interfires with Action class, must be disabled after usage
    MotionManager::GetInstance()->AddModule(static_cast<MotionModule *>(Walking::GetInstance()));
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

    while (true)
    {
        // Get distance and detection status from the head tracker
        double distance = head_tracker->GetDetectedObjectDistance();
        bool is_bottle_detected = (head_tracker->GetDetectedLabel() == "bottle" && distance > 0);

        // --- State Machine for Bottle Interaction ---
        switch (state)
        {
        case BottleTaskState::IDLE:
            if (is_bottle_detected)
            {
                std::cout << "INFO: New bottle detected. Starting approach." << std::endl;
                state = BottleTaskState::WALKING_TO_BOTTLE;
            }
            break;

        case BottleTaskState::WALKING_TO_BOTTLE:
        {
            // This threshold is critical and must be tuned carefully!
            const double PICKUP_DISTANCE_THRESHOLD = 0.25; // in meters

            if (!is_bottle_detected)
            {
                std::cout << "INFO: Lost sight of bottle, stopping walk." << std::endl;
                Walking::GetInstance()->Stop();
                MotionManager::GetInstance()->RemoveModule(static_cast<MotionModule *>(Walking::GetInstance()));
                MotionManager::GetInstance()->SetEnable(false); // Enable motion for walking

                follower.Process(Point2D(-1.0, -1.0)); // Tell follower no target
                state = BottleTaskState::IDLE;
                break;
            }

            // Check if we are close enough to pick up the bottle
            if (distance <= PICKUP_DISTANCE_THRESHOLD)
            {
                std::cout << "INFO: Reached bottle (" << distance << "m). Stopping walk and preparing for pickup." << std::endl;
                Walking::GetInstance()->Stop();
                MotionManager::GetInstance()->RemoveModule(static_cast<MotionModule *>(Walking::GetInstance()));
                MotionManager::GetInstance()->SetEnable(false); // Disable motion manager

                // IMPORTANT: Wait for the robot to become fully stationary
                while (Walking::GetInstance()->IsRunning())
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                std::cout << "INFO: Walk stopped. Transitioning to PICKING_UP state." << std::endl;
                state = BottleTaskState::PICKING_UP;
                break; // Exit and let the next loop iteration handle the PICKING_UP state
            }

            // If we are still too far, continue walking
            Point2D object_angular_error = head_tracker->GetLastDetectedObjectAngularError();
            if (object_angular_error.X != -1.0)
            {
                // This tells the Walking module how to adjust its steps
                follower.Process(object_angular_error);
            }
        }
        break;

        case BottleTaskState::PICKING_UP:
        {
            std::cout << "INFO: Performing pickup sequence." << std::endl;

            legs_controller.ReadyToPickUpItem();
            right_arm_controller.PositionHandAway();
            right_arm_controller.RotateWrist90Deg();
            right_arm_controller.OpenGripper();
            right_arm_controller.HandReach();
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            right_arm_controller.CloseGripper();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            right_arm_controller.HoldItem();
            std::this_thread::sleep_for(std::chrono::milliseconds(4000));
            right_arm_controller.OpenGripper();
            right_arm_controller.Default();
            legs_controller.Stand();

            current_action_label = "bottle_pickup_complete";
            last_action_time = current_time;
            state = BottleTaskState::DONE;
        }
        break;

        case BottleTaskState::DONE:
            return;
        }
    }
}

void handleGenericObjectDetected(const std::string &label, int action_page,
                                 std::string &current_action_label,
                                 std::chrono::steady_clock::time_point &last_action_time,
                                 int &detect_count_ref, // Pass by reference to reset
                                 const std::chrono::steady_clock::time_point &current_time)
{
    std::cout << "INFO: Detected " << label << " consistently. Playing action page " << action_page << std::endl;
    run_action(action_page);
    current_action_label = label;
    last_action_time = current_time;
    detect_count_ref = 0; // Reset counter
}

void handleNoTargetOrStandby(std::string &current_action_label,
                             std::chrono::steady_clock::time_point &last_action_time,
                             const std::chrono::steady_clock::time_point &current_time)
{
    std::cout << "INFO: No target detected. Returning to standby..." << std::endl;
    current_action_label = "standby";
    last_action_time = current_time;
}

BottleTaskState current_bottle_task_state = BottleTaskState::IDLE;

int main(void)
{
    printf("\n===== Head tracking with Object Detection via Unix Domain Socket (Multithreaded) =====\n\n"); //

    change_current_dir(); // Change current directory to executable's location //

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
    LinuxCamera::GetInstance()->Initialize(0);        // Initialize with device index 0
    LinuxCamera::GetInstance()->LoadINISettings(ini); //
    std::cout << "INFO: Camera initialized and settings loaded." << std::endl;

    // --- Initialize Motion Framework Components ---
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);

    // Get MotionManager and Action singletons
    MotionManager *motion_manager = MotionManager::GetInstance(); //
    Action *action_module = Action::GetInstance();                // Get Action singleton //

    if (motion_manager->Initialize(&cm730) == false)
    {
        std::cerr << "ERROR: Failed to initialize Motion Manager in main!" << std::endl;
        delete ini;
        return -1;
    }

    motion_manager->LoadINISettings(ini);
    motion_manager->AddModule((MotionModule *)action_module);

    MotionManager::GetInstance()->SetEnable(true);

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(motion_manager);
    motion_timer->Start();

    LeftArmController left_arm_controller(&cm730);
    RightArmController right_arm_controller(&cm730);
    LegsController legs_controller(&cm730);

    HeadTracking *head_tracker = HeadTracking::GetInstance();

    if (!head_tracker->Initialize(ini, &cm730)) // Updated call //
    {
        std::cerr << "ERROR: HeadTracking initialization failed. Exiting." << std::endl; //
        motion_timer->Stop();
        MotionManager::GetInstance()->SetEnable(false);
        MotionManager::GetInstance()->RemoveModule((MotionModule *)action_module);
        delete ini;
        delete motion_timer;
        return -1;
    }

    std::cout << "INFO: Playing initial standby action (Page " << ACTION_PAGE_STAND << ")..." << std::endl; //
    run_action(ACTION_PAGE_STAND);

    pthread_t tracking_thread;
    std::cout << "INFO: Creating HeadTracking thread..." << std::endl;                                   //
    int thread_create_status = pthread_create(&tracking_thread, NULL, HeadTrackingThread, head_tracker); //

    if (thread_create_status != 0)
    {
        std::cerr << "ERROR: Failed to create HeadTracking thread: " << strerror(thread_create_status) << std::endl; //
        head_tracker->Cleanup();                                                                                     //
        motion_timer->Stop();                                                                                        //
        MotionManager::GetInstance()->SetEnable(false);                                                              //
        MotionManager::GetInstance()->RemoveModule((MotionModule *)action_module);                                   //
        delete ini;                                                                                                  //
        delete motion_timer;                                                                                         //
        return -1;
    }
    std::cout << "INFO: HeadTracking thread created successfully." << std::endl; //

    std::cout << "INFO: Main thread running, checking for detected objects to trigger actions. Press Ctrl+C to exit." << std::endl; //

    std::string current_action_label = "standby";
    auto last_action_time = std::chrono::steady_clock::now();
    const auto action_cooldown = std::chrono::seconds(9);

    int person_detect_count = 0;
    int bottle_detect_count = 0;
    int dog_detect_count = 0;
    int cat_detect_count = 0;
    int sports_ball_detect_count = 0;
    // Add other detection counters here if needed

    const int detect_threshold = 10;

    while (1)
    {
        std::string detected_object_label = head_tracker->GetDetectedLabel();
        double distance = 0;

        if (detected_object_label != "none")
        {
            distance = head_tracker->GetDetectedObjectDistance(); //
            if (distance > 0)                                     //
            {
                std::cout << MAGENTA << "INFO: Estimated distance to " << detected_object_label
                          << ": " << distance << " meters." << RESET << std::endl;
            }
        }

        detected_object_label.erase(std::remove_if(detected_object_label.begin(), detected_object_label.end(), [](unsigned char c) //
                                                   { return std::isspace(c); }),
                                    detected_object_label.end());

        auto current_time = std::chrono::steady_clock::now();
        bool can_perform_action = (current_time - last_action_time) >= action_cooldown;

        // Update detection counts
        if (detected_object_label == "person")
        {
            person_detect_count++;
        }
        else
        {
            person_detect_count = 0;
        }
        if (detected_object_label == "bottle")
        {
            bottle_detect_count++;
        }
        else
        {
            bottle_detect_count = 0;
        }
        if (detected_object_label == "dog")
        {
            dog_detect_count++;
        }
        else
        {
            dog_detect_count = 0;
        }
        if (detected_object_label == "cat")
        {
            cat_detect_count++;
        }
        else
        {
            cat_detect_count = 0;
        }
        if (detected_object_label == "sports ball")
        {
            sports_ball_detect_count++;
        }
        else
        {
            sports_ball_detect_count = 0;
        }
        // Update other counters similarly

        // Action logic using encapsulated methods
        if (detected_object_label == "person" && person_detect_count >= detect_threshold && current_action_label != "person" && can_perform_action)
        {
            handlePersonDetected(left_arm_controller, current_action_label, last_action_time, person_detect_count, current_time);
        }
        else if (detected_object_label == "bottle" && bottle_detect_count >= detect_threshold && current_action_label != "bottle" && can_perform_action)
        {
            handleBottleInteraction(current_bottle_task_state,
                                    legs_controller,
                                    right_arm_controller,
                                    head_tracker,
                                    current_action_label,
                                    last_action_time,
                                    current_time);

            if (current_bottle_task_state == BottleTaskState::DONE)
            {
                std::cout << "Task complete! Resetting to IDLE in 5 seconds." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(5));
                current_bottle_task_state = BottleTaskState::IDLE;
            }
        }
        else if (detected_object_label == "dog" && dog_detect_count >= detect_threshold && current_action_label != "dog" && can_perform_action)
        {
            handleGenericObjectDetected("dog", ACTION_PAGE_DOG, current_action_label, last_action_time, dog_detect_count, current_time);
        }
        else if (detected_object_label == "cat" && cat_detect_count >= detect_threshold && current_action_label != "cat" && can_perform_action)
        {
            handleGenericObjectDetected("cat", ACTION_PAGE_CAT, current_action_label, last_action_time, cat_detect_count, current_time);
        }
        else if (detected_object_label == "sports ball" && sports_ball_detect_count >= detect_threshold && current_action_label != "sports ball" && can_perform_action)
        {
            handleGenericObjectDetected("sports ball", ACTION_PAGE_SPORTS_BALL, current_action_label, last_action_time, sports_ball_detect_count, current_time);
        }
        // Add other object handling "else if" blocks here, potentially using handleGenericObjectDetected
        // or new specific handlers if their logic is complex.

        else if (detected_object_label == "none" && current_action_label != "standby" && can_perform_action) //
        {
            handleNoTargetOrStandby(current_action_label, last_action_time, current_time);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(150)); //
    }

    std::cout << "INFO: Main loop terminated. Waiting for HeadTracking thread to join..." << std::endl; //
    pthread_join(tracking_thread, NULL);                                                                //
    std::cout << "INFO: HeadTracking thread joined." << std::endl;                                      //

    std::cout << "INFO: Shutting down motion framework..." << std::endl;       //
    motion_timer->Stop();                                                      //
    MotionManager::GetInstance()->SetEnable(false);                            //
    MotionManager::GetInstance()->RemoveModule((MotionModule *)action_module); //

    delete ini;
    delete motion_timer;

    std::cout << "INFO: Main program exiting." << std::endl; //

    return 0;
}