/*
 * main.cpp
 *
 * Debug Mode: Prints every detection state with automated gesture detector launch.
 */

#include "ConsoleColors.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <iostream>
#include <cstdlib>
#include <pthread.h>
#include <string>
#include <chrono>
#include <thread>
#include <signal.h>
#include <sys/stat.h>

#include "minIni.h"
#include "HeadTracking.h"
#include "LeftArmController.h"
#include "RightArmController.h"
#include "LegsController.h"
#include "LinuxDARwIn.h"

#define INI_FILE_PATH "../../../../Data/config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0"
#define MOTION_FILE_PATH "../../../../Data/motion_4096.bin"

#define ACTION_PAGE_WAVE 8
#define ACTION_PAGE_READY 1

#define DETECT_THRESHOLD 1 

using namespace Robot;

volatile bool is_running = true;

void cleanup_processes()
{
    system("pkill -9 -f gesture_detector.py 2>/dev/null");
    system("pkill -9 mjpg_streamer 2>/dev/null");
}

void sighandler(int sig)
{
    std::cout << "\n[INFO] Caught signal " << sig << ", shutting down gracefully..." << std::endl;
    is_running = false;
    cleanup_processes();
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
    while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);

    MotionManager::GetInstance()->AddModule(static_cast<MotionModule *>(Walking::GetInstance()));
    MotionManager::GetInstance()->SetEnable(false);
}

void *HeadTrackingThread(void *arg)
{
    HeadTracking *head_tracker = static_cast<HeadTracking *>(arg);
    if (head_tracker) head_tracker->Run();
    return NULL;
}

int main(void)
{
    printf("\n===== Darwin-OP Gesture Mode (Auto-Launch) =====\n\n");
    
    signal(SIGPIPE, SIG_IGN);
    change_current_dir();

    // 1. Terminate any previous instances
    cleanup_processes();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    minIni *ini = new minIni(INI_FILE_PATH);
    
    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);
    
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);

    MotionManager *motion_manager = MotionManager::GetInstance();
    Action *action_module = Action::GetInstance();
    action_module->LoadFile(MOTION_FILE_PATH);

    motion_manager->Initialize(&cm730);
    motion_manager->LoadINISettings(ini);
    motion_manager->AddModule((MotionModule *)action_module);

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(motion_manager);
    motion_timer->Start();

    LeftArmController left_arm_controller(&cm730);
    RightArmController right_arm_controller(&cm730);
    LegsController legs_controller(&cm730);

    HeadTracking *head_tracker = HeadTracking::GetInstance();
    if (!head_tracker->Initialize(ini, &cm730, 3)) return -1; 

    // 2. Set permissive socket permissions and launch gesture detector
    chmod("/tmp/darwin_detector.sock", 0666);
    std::cout << "[INFO] Launching gesture_detector.py..." << std::endl;
    system("python3 -u /home/darwin/darwin/aiy-maker-kit/python/gesture_detector.py &");

    std::cout << "INFO: Initial Pose..." << std::endl;
    run_action(ACTION_PAGE_READY);

    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    pthread_t tracking_thread;
    pthread_create(&tracking_thread, NULL, HeadTrackingThread, head_tracker);

    std::cout << "\033[1;32mREADY: Waiting for hand_wave...\033[0m" << std::endl;

    int wave_counter = 0;
    
    while (is_running)
    {
        std::string label = head_tracker->GetDetectedLabel();

        if (label != "none" && !label.empty()) {
             std::cout << "DEBUG: Main loop sees: " << label << std::endl;
        }

        if (label == "hand_wave")
        {
            wave_counter++;
            if (wave_counter >= DETECT_THRESHOLD)
            {
                std::cout << "\n\033[1;35m>>> WAVE ACTION TRIGGERED! <<<\033[0m" << std::endl;
                run_action(ACTION_PAGE_WAVE);
                wave_counter = 0;
                
                std::cout << "INFO: Back to Ready." << std::endl;
                run_action(ACTION_PAGE_READY);
                std::cout << "\033[1;32mREADY: Waiting for hand_wave...\033[0m" << std::endl;
            }
        } 
        else 
        {
             if (wave_counter > 0) wave_counter--; 
        } 
 
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    cleanup_processes();
    motion_timer->Stop();
    MotionManager::GetInstance()->SetEnable(false);

    return 0;
}