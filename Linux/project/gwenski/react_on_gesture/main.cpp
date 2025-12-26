/*
 * main.cpp
 * FINAL FIX
 * - Correctly initializes LinuxCM730 with "/dev/ttyUSB0"
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <iostream>
#include <cstdlib>
#include <pthread.h>
#include <string>
#include <thread>
#include <chrono>
#include <signal.h>

// Framework Headers
#include "HeadTracking.h"
#include "LinuxDARwIn.h"
#include "LinuxActionScript.h"

// --- CONFIGURATION ---
#define INI_FILE_PATH       "../../../../Data/config.ini"
#define MOTION_FILE_PATH    "../../../../Data/motion_4096.bin"
#define ACTION_PAGE_WAVE    15  // "Hello"
#define ACTION_PAGE_READY   9   // "Walk Ready"
#define DETECT_THRESHOLD    2   // Frames to confirm wave

using namespace Robot;

// --- HELPER FUNCTION ---
void performWaveAction() {
    std::cout << "\n\033[1;35m>>> WAVE DETECTED! Greeting human... \033[0m" << std::endl;
    
    if (Action::GetInstance()->IsRunning() == 0) {
        // Enable bodies and head
        Action::GetInstance()->m_Joint.SetEnableBody(true, true);
        
        // Start Wave
        Action::GetInstance()->Start(ACTION_PAGE_WAVE);
        
        // Wait for action to finish
        while (Action::GetInstance()->IsRunning()) {
            usleep(50000); 
        }
    }
}

int main(int argc, char* argv[]) {
    signal(SIGPIPE, SIG_IGN);
    std::cout << "\n\033[1;36m=== Darwin-OP Gesture Interaction (Initialized) ===\033[0m" << std::endl;

    // 1. SYSTEM INITIALIZATION
    // ---------------------------------------------------------
    minIni* ini = new minIni(INI_FILE_PATH);
    
    // FIX: Pass the device port explicitly
    LinuxCM730 linux_cm730("/dev/ttyUSB0"); 
    
    CM730 cm730(&linux_cm730);

    if (MotionManager::GetInstance()->Initialize(&cm730) == false) {
        std::cerr << "ERROR: Fail to initialize Motion Manager!" << std::endl;
        return 0;
    }

    // 2. Initialize HeadTracking (Pass hardware pointers)
    if (HeadTracking::GetInstance()->Initialize(ini, &cm730) == false) {
        std::cerr << "ERROR: Fail to initialize HeadTracking!" << std::endl;
        return 0;
    }

    // 3. Load Motion File
    if (Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH) == false) {
        std::cerr << "ERROR: Fail to load Motion file!" << std::endl;
        return 0;
    }
    // ---------------------------------------------------------

    // 4. Setup Motion Timer
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    // 5. Start Vision Thread
    std::cout << "INFO: Launching Vision Thread..." << std::endl;
    pthread_t tracking_thread;
    if (pthread_create(&tracking_thread, NULL, HeadTracking::AutoTrackingLoop, NULL) != 0) {
        std::cerr << "ERROR: Failed to create HeadTracking thread" << std::endl;
        return -1;
    }

    // 6. Robot Stand Up
    std::cout << "INFO: Robot Standing Up..." << std::endl;
    MotionManager::GetInstance()->SetEnable(true);
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    Action::GetInstance()->Start(ACTION_PAGE_READY); 
    while (Action::GetInstance()->IsRunning()) usleep(8000);

    std::cout << "\033[1;32mINFO: Ready! Waiting for wave...\033[0m" << std::endl;

    // 7. Main Loop
    int wave_counter = 0;

    while (true) {
        // Safe access to label
        std::string label = HeadTracking::GetInstance()->GetDetectedLabel();

        if (label == "hand_wave") {
            wave_counter++;
            std::cout << "DEBUG: Wave Validating... " << wave_counter << "/" << DETECT_THRESHOLD << "\r" << std::flush;

            if (wave_counter >= DETECT_THRESHOLD) {
                performWaveAction();
                
                // Reset
                wave_counter = 0;
                Action::GetInstance()->Start(ACTION_PAGE_READY); 
                while (Action::GetInstance()->IsRunning()) usleep(8000);
                std::cout << "INFO: Ready for next gesture.      " << std::endl;
            }
        } else {
            if (wave_counter > 0) wave_counter--;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Cleanup
    pthread_join(tracking_thread, NULL);
    motion_timer->Stop();
    MotionManager::GetInstance()->SetEnable(false);
    return 0;
}