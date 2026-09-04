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
#include <csignal>

#include "minIni.h" // For INI file loading
#include "HeadTracking.h"
#include "LeftArmController.h"
#include "RightArmController.h"
#include "LegsController.h"
#include "LinuxDARwIn.h" // Include for Motion Framework components (MotionManager, Action)
#include "LinuxActionScript.h"
#include "VoiceCommander.h"

// --- Configuration ---
#define INI_FILE_PATH "../../../../Data/config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0" // Verify this path is correct!
#define MOTION_FILE_PATH "../../../../Data/motion_4096.bin"

// Define action page numbers for different detected objects
#define ACTION_PAGE_WAVE 7
#define ACTION_PAGE_WAVE2 8
#define ACTION_PAGE_WAVE3 4
#define ACTION_PAGE_HAPPY 14
#define ACTION_PAGE_CAT 12
#define ACTION_PAGE_SPORTS_BALL 13
#define ACTION_PAGE_STAND 1
#define ACTION_PAGE_READY_TO_PICKUP 32
#define ACTION_PAGE_PICKUP_ITEM 33
#define ACTION_PAGE_PASS_ITEM 34
#define ACTION_PAGE_HOLD_ITEM 35

enum class BottleTaskState
{
    IDLE,
    WALKING_TO_BOTTLE,
    PICKING_UP,
    DONE
};

void set_enable_motion_manager_and_walking(bool enable)
{
    if (enable)
    {
        MotionManager::GetInstance()->AddModule(static_cast<MotionModule *>(Walking::GetInstance()));
        Walking::GetInstance()->BALANCE_ENABLE = true;
        MotionManager::GetInstance()->SetEnable(true);
    }
    else
    {
        MotionManager::GetInstance()->RemoveModule(static_cast<MotionModule *>(Walking::GetInstance()));
        MotionManager::GetInstance()->SetEnable(false);
    }
}

void change_current_dir()
{
    char exepath[1024] = {0};
    if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void run_action(int action_page)
{
    MotionManager::GetInstance()->RemoveModule(static_cast<MotionModule *>(Walking::GetInstance()));
    MotionManager::GetInstance()->SetEnable(true);

    MotionManager::GetInstance()->SetJointEnableState(JointData::ID_HEAD_PAN, false);
    MotionManager::GetInstance()->SetJointEnableState(JointData::ID_HEAD_TILT, false);

    Action::GetInstance()->Start(action_page);

    while (Action::GetInstance()->IsRunning())
        usleep(8 * 1000);

    MotionManager::GetInstance()->AddModule(static_cast<MotionModule *>(Walking::GetInstance()));
    MotionManager::GetInstance()->SetEnable(false);
}

void run_action_non_blocking(int action_page)
{
    MotionManager::GetInstance()->RemoveModule(static_cast<MotionModule *>(Walking::GetInstance()));
    MotionManager::GetInstance()->SetEnable(true);

    MotionManager::GetInstance()->SetJointEnableState(JointData::ID_HEAD_PAN, false);
    MotionManager::GetInstance()->SetJointEnableState(JointData::ID_HEAD_TILT, false);

    Action::GetInstance()->Start(action_page);
}

void *HeadTrackingThread(void *arg)
{
    HeadTracking *head_tracker = static_cast<HeadTracking *>(arg);
    if (head_tracker)
    {
        head_tracker->Run();
    }
    else
    {
        std::cerr << "ERROR: HeadTrackingThread received a null pointer." << std::endl;
    }
    return NULL;
}

void handlePersonDetected(LeftArmController &left_arm_controller,
                          std::string &current_action_label,
                          std::chrono::steady_clock::time_point &last_action_time,
                          int &person_detect_count_ref,
                          const std::chrono::steady_clock::time_point &current_time)
{
    std::cout << "INFO: Detected person consistently. Playing Wave" << std::endl;

    int random = rand() % 3;
    if (random == 0)
    {
        system("espeak \"Hello there\" &");
        run_action(ACTION_PAGE_WAVE);
    }
    else if (random == 1)
    {
        system("espeak \"I can see you\" &");
        run_action(ACTION_PAGE_WAVE2);
    }
    else
    {
        system("espeak \"Hello\" &");
        run_action(ACTION_PAGE_WAVE3);
    }

    run_action(ACTION_PAGE_STAND);

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
    set_enable_motion_manager_and_walking(true);
    MotionManager::GetInstance()->SetJointEnableState(JointData::ID_HEAD_TILT, false);
    MotionManager::GetInstance()->SetJointEnableState(JointData::ID_HEAD_PAN, false);

    while (true)
    {
        // Get distance and detection status from the head tracker
        double distance = head_tracker->GetDetectedObjectDistance();
        std::cout << CYAN << "INFO: Detected object distance: " << distance << "m" << RESET << std::endl;

        bool is_bottle_detected = (head_tracker->GetDetectedLabel() == "bottle" && distance > 0);

        // --- State Machine for Bottle Interaction ---
        switch (state)
        {
        case BottleTaskState::IDLE:
            if (is_bottle_detected)
            {
                std::cout << GREEN << "INFO: New bottle detected. Starting approach." << RESET << std::endl;
                state = BottleTaskState::WALKING_TO_BOTTLE;
            }
            else
            {
                run_action(ACTION_PAGE_STAND);
            }

            break;

        case BottleTaskState::WALKING_TO_BOTTLE:
        {
            // This threshold is critical and must be tuned carefully!
            const double PICKUP_DISTANCE_THRESHOLD = 0.30; // in meters

            if (!is_bottle_detected)
            {
                std::cout << "INFO: Lost sight of bottle, stopping walk." << std::endl;
                Walking::GetInstance()->Stop();

                follower.Process(Point2D(-1.0, -1.0)); // Tell follower no target
                state = BottleTaskState::IDLE;
                break;
            }

            // Check if we are close enough to pick up the bottle
            if (distance <= PICKUP_DISTANCE_THRESHOLD)
            {
                std::cout << "INFO: Reached bottle (" << distance << "m). Stopping walk and preparing for pickup." << std::endl;
                Walking::GetInstance()->Stop();

                // IMPORTANT: Wait for the robot to become fully stationary
                while (Walking::GetInstance()->IsRunning())
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                std::cout << "INFO: Walk stopped. Transitioning to PICKING_UP state." << std::endl;
                state = BottleTaskState::PICKING_UP;
                break;
            }

            // If we are still too far, continue walking
            Point2D object_angular_error = head_tracker->GetLastDetectedObjectAngularError();
            follower.Process(object_angular_error);
        }
        break;

        case BottleTaskState::PICKING_UP:
        {
            std::cout << GREEN << "INFO: Performing pickup sequence." << RESET << std::endl;

            MotionManager::GetInstance()->RemoveModule(static_cast<MotionModule *>(Walking::GetInstance()));
            run_action(ACTION_PAGE_READY_TO_PICKUP);
            set_enable_motion_manager_and_walking(false);

            right_arm_controller.OpenGripper();
            // right_arm_controller.CenterHandInView();

            run_action(ACTION_PAGE_PICKUP_ITEM);
            std::this_thread::sleep_for(std::chrono::milliseconds(4000));

            run_action(ACTION_PAGE_PASS_ITEM);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            run_action(ACTION_PAGE_STAND);
            set_enable_motion_manager_and_walking(true);

            current_action_label = "bottle_pickup_complete";
            last_action_time = current_time;
            state = BottleTaskState::DONE;
        }
        break;

        case BottleTaskState::DONE:
            set_enable_motion_manager_and_walking(false);
            return;
        }
    }
}

void handleGenericObjectDetected(const std::string &label, int action_page,
                                 std::string &current_action_label,
                                 std::chrono::steady_clock::time_point &last_action_time,
                                 int &detect_count_ref,
                                 const std::chrono::steady_clock::time_point &current_time)
{
    std::cout << "INFO: Detected " << label << " consistently. Playing action page " << action_page << std::endl;

    if (label == "dog")
    {
        system("espeak \"Such a nice doggy\" &");
        run_action(action_page);
    }
    else if (label == "cat")
    {
        system("espeak \"Here kitty kitty\" &");
        run_action(action_page);
    }
    else if (label == "sports_ball")
    {
        system("espeak \"Let's play ball\" &");
        run_action(action_page);
    }

    run_action(ACTION_PAGE_STAND);
    current_action_label = label;
    last_action_time = current_time;
    detect_count_ref = 0;
}

void handleNoTargetOrStandby(std::string &current_action_label,
                             std::chrono::steady_clock::time_point &last_action_time,
                             const std::chrono::steady_clock::time_point &current_time)
{
    std::cout << "INFO: No target detected. Returning to standby..." << std::endl;
    current_action_label = "standby";
    last_action_time = current_time;
}

void sigint_handler(int sig)
{
    std::cout << "\n\nINFO: Shutting down safely..." << std::endl;

    system("espeak \"Shutting down safely\" &");

    system("pkill -2 -f voice_listener.py");
    system("pkill -f custom_detect_objects.py");
    system("pkill mjpg_streamer");

    MotionManager::GetInstance()->SetEnable(false);

    std::cout << "INFO: Background scripts killed and motors relaxed. Goodbye!" << std::endl;
    exit(0);
}

BottleTaskState current_bottle_task_state = BottleTaskState::IDLE;

void RegisterAllVoiceCommands(VoiceCommander& voice, 
                              LeftArmController& left_arm_controller, 
                              RightArmController& right_arm_controller, 
                              bool& is_holding_item, 
                              std::string& current_action_label, 
                              std::chrono::steady_clock::time_point& last_action_time, 
                              int& bottle_detect_count) 
{
    // 1. System Commands
    auto exit_action = []() { sigint_handler(SIGINT); };
    voice.RegisterCommand("спри", exit_action);
    voice.RegisterCommand("спи", exit_action);
    voice.RegisterCommand("изключи", exit_action);
    voice.RegisterCommand("край", exit_action);

    // 2. Greetings
    auto greet_action = [&]() {
        if (is_holding_item) {
            system("espeak -v bg \"Здрасти. В момента държа нещо.\" &");
        } else {
            system("espeak -v bg \"Здравей\" &");
            int wave_pages[3] = {ACTION_PAGE_WAVE3, ACTION_PAGE_WAVE, ACTION_PAGE_WAVE2}; 
            run_action(wave_pages[rand() % 3]);
            run_action(ACTION_PAGE_STAND);
            current_action_label = "standby";
            last_action_time = std::chrono::steady_clock::now();
        }
    };
    voice.RegisterCommand("здравей", greet_action);
    voice.RegisterCommand("здрасти", greet_action);
    voice.RegisterCommand("хей", greet_action);

    // 3. Stand / Reset
    auto stand_action = [&]() {
        std::cout << GREEN << "INFO: Returning to stand position..." << RESET << std::endl;
        system("espeak -v bg \"Изправям се\" &");
        run_action(ACTION_PAGE_STAND);
        Action::GetInstance()->m_Joint.SetEnable(22, true);
        Action::GetInstance()->m_Joint.SetEnable(24, true);
        current_action_label = "standby";
        last_action_time = std::chrono::steady_clock::now();
        bottle_detect_count = 0;
        is_holding_item = false;
    };
    voice.RegisterCommand("изправи се", stand_action);
    voice.RegisterCommand("център", stand_action);

    // 4. Independent Grippers
    voice.RegisterCommand("отвори лява", [&]() {
        system("espeak -v bg \"Отварям лявата\" &");
        Action::GetInstance()->m_Joint.SetEnable(24, false);
        left_arm_controller.OpenGripper();
    });
    
    voice.RegisterCommand("затвори лява", [&]() {
        system("espeak -v bg \"Затварям лявата\" &");
        Action::GetInstance()->m_Joint.SetEnable(24, false);
        left_arm_controller.CloseGripper();
    });

    voice.RegisterCommand("отвори дясна", [&]() {
        system("espeak -v bg \"Отварям дясната\" &");
        Action::GetInstance()->m_Joint.SetEnable(22, false);
        right_arm_controller.OpenGripper();
    });

    voice.RegisterCommand("затвори дясна", [&]() {
        system("espeak -v bg \"Затварям дясната\" &");
        Action::GetInstance()->m_Joint.SetEnable(22, false);
        right_arm_controller.CloseGripper();
    });

    // 5. Holding Item Workflows
    auto hold_action = [&]() {
        system("espeak -v bg \"Държа\" &");
        run_action(ACTION_PAGE_HOLD_ITEM);
        Action::GetInstance()->m_Joint.SetEnable(22, false);
        right_arm_controller.OpenGripper();
        current_action_label = "bottle";
        last_action_time = std::chrono::steady_clock::now();
        is_holding_item = true; 
    };
    voice.RegisterCommand("дръж", hold_action);
    voice.RegisterCommand("хвани", hold_action);
    voice.RegisterCommand("вземи", hold_action);

    auto release_action = [&]() {
        if (is_holding_item) {
            system("espeak -v bg \"Пускам\" &");
            Action::GetInstance()->m_Joint.SetEnable(22, false);
            right_arm_controller.OpenGripper();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            run_action(ACTION_PAGE_STAND);
            Action::GetInstance()->m_Joint.SetEnable(22, true);
            
            current_action_label = "standby";
            last_action_time = std::chrono::steady_clock::now();
            bottle_detect_count = 0;
            is_holding_item = false; 
        }
    };
    voice.RegisterCommand("пусни", release_action);
    voice.RegisterCommand("остави", release_action);

    // 6. Generic Close (Closes both grippers)
    auto close_action = [&]() {
        system("espeak -v bg \"Затварям\" &"); 
        Action::GetInstance()->m_Joint.SetEnable(22, false);
        Action::GetInstance()->m_Joint.SetEnable(24, false);
        right_arm_controller.CloseGripper(); 
        left_arm_controller.CloseGripper();
    };
    
    // Note: Generic matches must go AFTER specific matches in the sequence
    voice.RegisterCommand("затвори двете", close_action);
    voice.RegisterCommand("затвори всичко", close_action);
    voice.RegisterCommand("затвори", close_action);
}

int main(void)
{
    signal(SIGINT, sigint_handler);

    srand(time(NULL));
    change_current_dir();

    std::cout << "Cleaning up stale processes..." << std::endl;
    system("pkill -f custom_detect_object.py");
    system("pkill mjpg_streamer");
    system("pkill -2 -f voice_listener.py");

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "INFO: Starting background voice listener..." << std::endl;
    system("sudo -u darwin python3 -u /home/darwin/darwin/Linux/project/gwenski/react_on_object_detect/voice_listener.py 2>/dev/null &");

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    minIni *ini = new minIni(INI_FILE_PATH);
    if (!ini)
    {
        std::cerr << "ERROR: Failed to load INI file." << std::endl;
        system("espeak \"Fatal Error. Failed to load configuration file.\"");
        return -1;
    }

    Robot::Action::GetInstance()->LoadFile((char *)MOTION_FILE_PATH);

    std::cout << "INFO: Initializing camera..." << std::endl;
    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);
    std::cout << "INFO: Camera initialized and settings loaded." << std::endl;

    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);

    MotionManager *motion_manager = MotionManager::GetInstance();
    Action *action_module = Action::GetInstance();

    if (motion_manager->Initialize(&cm730) == false)
    {
        std::cerr << "ERROR: Failed to initialize Motion Manager in main!" << std::endl;
        system("espeak \"Fatal Error. Servo connection failed.\"");
        delete ini;
        return -1;
    }

    motion_manager->LoadINISettings(ini);
    motion_manager->AddModule((MotionModule *)action_module);

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(motion_manager);
    motion_timer->Start();

    LeftArmController left_arm_controller(&cm730);
    RightArmController right_arm_controller(&cm730);
    LegsController legs_controller(&cm730);

    HeadTracking *head_tracker = HeadTracking::GetInstance();

    if (!head_tracker->Initialize(ini, &cm730))
    {
        std::cerr << "ERROR: HeadTracking initialization failed. Exiting." << std::endl;
        system("espeak \"Fatal Error. Camera initialization failed.\"");
        motion_timer->Stop();
        MotionManager::GetInstance()->SetEnable(false);
        MotionManager::GetInstance()->RemoveModule((MotionModule *)action_module);
        delete ini;
        delete motion_timer;
        return -1;
    }

    std::cout << "INFO: Playing initial standby action (Page " << ACTION_PAGE_STAND << ")..." << std::endl;
    run_action(ACTION_PAGE_STAND);

    pthread_t tracking_thread;
    std::cout << "INFO: Creating HeadTracking thread..." << std::endl;
    int thread_create_status = pthread_create(&tracking_thread, NULL, HeadTrackingThread, head_tracker);

    if (thread_create_status != 0)
    {
        std::cerr << "ERROR: Failed to create HeadTracking thread: " << strerror(thread_create_status) << std::endl;
        system("espeak \"Fatal Error. Failed to launch background vision process.\"");
        head_tracker->Cleanup();
        motion_timer->Stop();
        MotionManager::GetInstance()->SetEnable(false);
        MotionManager::GetInstance()->RemoveModule((MotionModule *)action_module);
        delete ini;
        delete motion_timer;
        return -1;
    }
    std::cout << "INFO: HeadTracking thread created successfully." << std::endl;

    VoiceCommander voice;

    //=========================================================================
    // VOICE STARTUP SEQUENCE
    //=========================================================================
    system("espeak -v bg \"Инициализацията завърши. Чакам команда за старт.\" &");
    bool start_command_received = false;
    while (!start_command_received)
    {
        std::string cmd = voice.GetRawCommand();
        if (cmd.find("старт") != std::string::npos || cmd.find("започни") != std::string::npos || cmd.find("тръгвай") != std::string::npos)
        {
            std::cout << GREEN << "INFO: Start command received: '" << cmd << "'" << RESET << std::endl;
            std::string speak_cmd = "espeak -v bg \"" + cmd + "\" &";
            system(speak_cmd.c_str());
            start_command_received = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    //=========================================================================

    std::string current_action_label = "standby";
    auto last_action_time = std::chrono::steady_clock::now();
    const auto action_cooldown = std::chrono::seconds(9);

    int person_detect_count = 0;
    int bottle_detect_count = 0;
    int dog_detect_count = 0;
    int cat_detect_count = 0;
    int sports_ball_detect_count = 0;

    // NEW: Hard lock state for holding items
    bool is_holding_item = false;
    const int detect_threshold = 4;

    //=========================================================================
    // REGISTER VOICE COMMAND ACTIONS
    //=========================================================================
    RegisterAllVoiceCommands(voice, left_arm_controller, right_arm_controller, 
                             is_holding_item, current_action_label, 
                             last_action_time, bottle_detect_count);

    while (1)
    {
        std::string detected_object_label = head_tracker->GetDetectedLabel();
        double distance = 0;

        if (detected_object_label != "none")
        {
            distance = head_tracker->GetDetectedObjectDistance();
            if (distance > 0)
            {
                std::cout << MAGENTA << "INFO: Estimated distance to " << detected_object_label
                          << ": " << distance << " meters." << RESET << std::endl;
            }
        }

        detected_object_label.erase(std::remove_if(detected_object_label.begin(), detected_object_label.end(), [](unsigned char c)
                                                   { return std::isspace(c); }),
                                    detected_object_label.end());

        auto current_time = std::chrono::steady_clock::now();
        bool can_perform_action = (current_time - last_action_time) >= action_cooldown;

        if (detected_object_label == "person")
            person_detect_count++;
        else if (person_detect_count > 0)
            person_detect_count--;
        if (detected_object_label == "bottle")
            bottle_detect_count++;
        else if (bottle_detect_count > 0)
            bottle_detect_count--;
        if (detected_object_label == "dog")
            dog_detect_count++;
        else if (dog_detect_count > 0)
            dog_detect_count--;
        if (detected_object_label == "cat")
            cat_detect_count++;
        else if (cat_detect_count > 0)
            cat_detect_count--;
        if (detected_object_label == "sportsball")
            sports_ball_detect_count++;
        else if (sports_ball_detect_count > 0)
            sports_ball_detect_count--;

        // =========================================================================
        // --- VISION TRIGGERS (ONLY IF NOT HOLDING AN ITEM) ---
        // =========================================================================
        if (!is_holding_item)
        {
            if (detected_object_label == "person" && person_detect_count >= detect_threshold && current_action_label != "person" && can_perform_action)
            {
                handlePersonDetected(left_arm_controller, current_action_label, last_action_time, person_detect_count, current_time);
            }
            else if (detected_object_label == "dog" && dog_detect_count >= detect_threshold && current_action_label != "dog" && can_perform_action)
            {
                handleGenericObjectDetected("dog", ACTION_PAGE_HAPPY, current_action_label, last_action_time, dog_detect_count, current_time);
            }
            else if (detected_object_label == "cat" && cat_detect_count >= detect_threshold && current_action_label != "cat" && can_perform_action)
            {
                handleGenericObjectDetected("cat", ACTION_PAGE_HAPPY, current_action_label, last_action_time, cat_detect_count, current_time);
            }
            else if (detected_object_label == "sportsball" && sports_ball_detect_count >= detect_threshold && current_action_label != "sports ball" && can_perform_action)
            {
                handleGenericObjectDetected("sports ball", ACTION_PAGE_SPORTS_BALL, current_action_label, last_action_time, sports_ball_detect_count, current_time);
            }
            else if (detected_object_label == "bottle" && bottle_detect_count >= detect_threshold && current_action_label != "bottle" && can_perform_action)
            {
                std::cout << GREEN << "INFO: Detected bottle visually. Playing hold action." << RESET << std::endl;
                run_action_non_blocking(ACTION_PAGE_HOLD_ITEM);

                // Open gripper to wait for the bottle
                Action::GetInstance()->m_Joint.SetEnable(22, false);
                right_arm_controller.OpenGripper();

                system("espeak \"Holding\" &");
                current_action_label = "bottle";
                last_action_time = current_time;
                bottle_detect_count = 0;
                is_holding_item = true; // Lock state
            }
            else if (bottle_detect_count == 0 && person_detect_count == 0 && dog_detect_count == 0 && cat_detect_count == 0 && sports_ball_detect_count == 0 && current_action_label != "standby" && can_perform_action)
            {
                std::cout << YELLOW << "INFO: Target lost. Returning to standby." << RESET << std::endl;
                run_action(ACTION_PAGE_STAND);
                current_action_label = "standby";
                last_action_time = current_time;
            }
            else if (detected_object_label == "none" && current_action_label != "standby" && can_perform_action)
            {
                handleNoTargetOrStandby(current_action_label, last_action_time, current_time);
            }
        }

        // =========================================================================
        // --- VOICE COMMAND PROCESSING BLOCK ---
        // =========================================================================
      std::ifstream voice_cmd_file("/tmp/darwin_voice_cmd.txt");
        if (voice_cmd_file.is_open())
        {
            std::string cmd;
            std::getline(voice_cmd_file, cmd);
            voice_cmd_file.close();

            if (!cmd.empty())
            {
                std::cout << GREEN << "INFO: Processing voice command: '" << cmd << "'" << RESET << std::endl;

                // 1. STOP / SLEEP COMMAND
                if (cmd.find("stop") != std::string::npos || cmd.find("sleep") != std::string::npos || cmd.find("quit") != std::string::npos || cmd.find("exit") != std::string::npos)
                {
                    std::remove("/tmp/darwin_voice_cmd.txt");
                    sigint_handler(SIGINT);
                }
                // 2. STAND / CENTER / DEFAULT
                else if (cmd.find("stand") != std::string::npos || cmd.find("center") != std::string::npos || cmd.find("default") != std::string::npos)
                {
                    std::cout << GREEN << "INFO: Returning to stand position..." << RESET << std::endl;
                    system("espeak \"Standing\" &");

                    run_action(ACTION_PAGE_STAND);

                    // Restore joint control to the Action framework for both grippers
                    Action::GetInstance()->m_Joint.SetEnable(22, true);
                    Action::GetInstance()->m_Joint.SetEnable(24, true);

                    current_action_label = "standby";
                    last_action_time = current_time;
                    bottle_detect_count = 0;
                    is_holding_item = false; // Reset any holding state
                }
                // 3. INDEPENDENT GRIPPER COMMANDS
                else if (cmd.find("open left") != std::string::npos)
                {
                    std::cout << GREEN << "INFO: Opening left gripper..." << RESET << std::endl;
                    system("espeak \"Opening left\" &");
                    Action::GetInstance()->m_Joint.SetEnable(24, false); // ID 24 is Left Gripper
                    left_arm_controller.OpenGripper();
                }
                else if (cmd.find("close left") != std::string::npos)
                {
                    std::cout << GREEN << "INFO: Closing left gripper..." << RESET << std::endl;
                    system("espeak \"Closing left\" &");
                    Action::GetInstance()->m_Joint.SetEnable(24, false);
                    left_arm_controller.CloseGripper();
                }
                else if (cmd.find("open right") != std::string::npos)
                {
                    std::cout << GREEN << "INFO: Opening right gripper..." << RESET << std::endl;
                    system("espeak \"Opening right\" &");
                    Action::GetInstance()->m_Joint.SetEnable(22, false); // ID 22 is Right Gripper
                    right_arm_controller.OpenGripper();
                }
                else if (cmd.find("close right") != std::string::npos)
                {
                    std::cout << GREEN << "INFO: Closing right gripper..." << RESET << std::endl;
                    system("espeak \"Closing right\" &");
                    Action::GetInstance()->m_Joint.SetEnable(22, false);
                    right_arm_controller.CloseGripper();
                }
                // 4. GREETING COMMAND (hi / hello / hey)
                else if (cmd.find("hi") != std::string::npos || cmd.find("hello") != std::string::npos || cmd.find("hey") != std::string::npos)
                {
                    std::cout << CYAN << "INFO: Greeting recognized." << RESET << std::endl;
                    
                    if (is_holding_item) {
                        system("espeak \"Hey what's up. I am currently holding something.\" &");
                    } else {
                        system("espeak -v bg \"опа ко става\" &");
                        int wave_pages[3] = {ACTION_PAGE_WAVE3, ACTION_PAGE_WAVE, ACTION_PAGE_WAVE2}; 
                        int chosen_wave = wave_pages[rand() % 3];

                        run_action(chosen_wave);
                        run_action(ACTION_PAGE_STAND);

                        current_action_label = "standby";
                        last_action_time = current_time;
                    }
                }
                // 5. HOLD / CATCH COMMAND
                else if (cmd.find("hold") != std::string::npos || cmd.find("catch") != std::string::npos || cmd.find("grab") != std::string::npos || cmd.find("take") != std::string::npos)
                {
                    std::cout << GREEN << "INFO: Executing Hold Item action..." << RESET << std::endl;
                    system("espeak \"Holding\" &");

                    run_action(ACTION_PAGE_HOLD_ITEM);

                    Action::GetInstance()->m_Joint.SetEnable(22, false);
                    right_arm_controller.OpenGripper();

                    current_action_label = "bottle";
                    last_action_time = current_time;
                    is_holding_item = true; 
                }
                // 6. GENERIC CLOSE COMMAND (For the right hand while holding)
                else if (cmd.find("close") != std::string::npos || cmd.find("shut") != std::string::npos)
                {
                    if (is_holding_item) 
                    {
                        std::cout << GREEN << "INFO: Closing gripper..." << RESET << std::endl;
                        system("espeak \"Closing\" &");

                        Action::GetInstance()->m_Joint.SetEnable(22, false);
                        right_arm_controller.CloseGripper(); 
                    }
                    else 
                    {
                        std::cout << YELLOW << "INFO: Ignored general close command (not holding anything). Try 'close right' or 'close left'." << RESET << std::endl;
                    }
                }
                // 7. GENERIC RELEASE COMMAND
                else if (cmd.find("release") != std::string::npos || cmd.find("drop") != std::string::npos || cmd.find("let go") != std::string::npos)
                {
                    if (is_holding_item) 
                    {
                        std::cout << GREEN << "INFO: Dropping item and standing..." << RESET << std::endl;
                        system("espeak \"Dropping\" &");

                        Action::GetInstance()->m_Joint.SetEnable(22, false);
                        right_arm_controller.OpenGripper();

                        std::this_thread::sleep_for(std::chrono::seconds(2));

                        run_action(ACTION_PAGE_STAND);
                        Action::GetInstance()->m_Joint.SetEnable(22, true);

                        current_action_label = "standby";
                        last_action_time = current_time;
                        bottle_detect_count = 0;
                        is_holding_item = false; 
                    }
                    else 
                    {
                        std::cout << YELLOW << "INFO: Ignored release command (not holding anything)." << RESET << std::endl;
                    }
                }

                // Consume and clear the file
                std::remove("/tmp/darwin_voice_cmd.txt");
            }
        }
        // =========================================================================

        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    std::cout << "INFO: Main loop terminated. Waiting for HeadTracking thread to join..." << std::endl;
    pthread_join(tracking_thread, NULL);
    std::cout << "INFO: HeadTracking thread joined." << std::endl;

    std::cout << "INFO: Shutting down motion framework..." << std::endl;
    motion_timer->Stop();
    MotionManager::GetInstance()->SetEnable(false);
    MotionManager::GetInstance()->RemoveModule((MotionModule *)action_module);

    delete ini;
    delete motion_timer;

    std::cout << "INFO: Main program exiting." << std::endl;

    return 0;
}