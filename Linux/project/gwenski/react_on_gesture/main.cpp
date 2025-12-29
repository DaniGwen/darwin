#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <pthread.h>
#include <string>
#include <thread>
#include <chrono>
#include <signal.h>

#include "HeadTracking.h"
#include "LinuxDARwIn.h"
#include "LinuxActionScript.h"

#define INI_FILE_PATH "../../../../Data/config.ini"
#define MOTION_FILE_PATH "../../../../Data/motion_4096.bin"
#define ACTION_PAGE_WAVE 15
#define ACTION_PAGE_READY 9
#define DETECT_THRESHOLD 2

using namespace Robot;

void performWaveAction()
{
    std::cout << "\n\033[1;35m>>> WAVE DETECTED! \033[0m" << std::endl;
    if (Action::GetInstance()->IsRunning() == 0)
    {
        Action::GetInstance()->m_Joint.SetEnableBody(true, true);
        Action::GetInstance()->Start(ACTION_PAGE_WAVE);
        while (Action::GetInstance()->IsRunning())
            usleep(10000);
    }
}

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

int main(int argc, char *argv[])
{

    std::cout << "Cleaning up stale processes..." << std::endl;
    system("pkill -9 -f gesture_detector.py");
    system("pkill -9 mjpg_streamer");

    signal(SIGPIPE, SIG_IGN);

    std::cout << "\n=== Darwin-OP Gesture Mode ===\n"
              << std::endl;

    // 1. Hardware Init
    minIni *ini = new minIni(INI_FILE_PATH);
    LinuxCM730 linux_cm730("/dev/ttyUSB0");
    CM730 cm730(&linux_cm730);

    if (MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        std::cerr << "Fail to init Motion Manager!" << std::endl;
        return 0;
    }

    HeadTracking *head_tracker = HeadTracking::GetInstance();
    head_tracker->Initialize(ini, &cm730);
    Action::GetInstance()
        ->LoadFile((char *)MOTION_FILE_PATH);

    // 2. Start Motion Thread
    MotionManager::GetInstance()->AddModule((MotionModule *)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule *)Head::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    // 3. Start Vision Thread
    pthread_t tracking_thread;
    pthread_create(&tracking_thread, NULL, HeadTrackingThread, head_tracker);

    // 4. Stand Up
    MotionManager::GetInstance()->SetEnable(true);
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    Action::GetInstance()->Start(ACTION_PAGE_READY);
    while (Action::GetInstance()->IsRunning())
        usleep(10000);

    std::cout << "\033[1;32mREADY: Waiting for hand_wave...\033[0m" << std::endl;

    // 5. Main Loop
    int wave_counter = 0;

    while (true)
    {
        std::string label = HeadTracking::GetInstance()->GetDetectedLabel();

        if (label == "hand_wave")
        {
            wave_counter++;
            if (wave_counter >= DETECT_THRESHOLD)
            {
                performWaveAction();
                wave_counter = 0;

                // Return to ready
                Action::GetInstance()->Start(ACTION_PAGE_READY);
                while (Action::GetInstance()->IsRunning())
                    usleep(20000);
            }
        }
        else
        {
            if (wave_counter > 0)
                wave_counter--;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}