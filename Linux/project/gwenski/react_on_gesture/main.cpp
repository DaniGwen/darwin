/*
 * main.cpp
 * SAFE VERSION (No hacks, no segfaults)
 * - Relies on HeadTracking to handle the camera and Python.
 * - Uses the new GetDetectedLabel() function we just added.
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <pthread.h>
#include <string>
#include <chrono>
#include <thread>

// Framework Headers
#include "HeadTracking.h"
#include "LinuxDARwIn.h"
#include "LinuxActionScript.h"

using namespace Robot;

// --- CONFIGURATION ---
#define MOTION_FILE_PATH   "../../../../Data/motion_4096.bin"
#define ACTION_PAGE_WAVE   15  // Page 15 is standard "Hello"
#define ACTION_PAGE_READY  9   // Page 9 is "Walk Ready"
#define DETECT_THRESHOLD   2   // Frames needed to confirm wave

// --- GLOBAL ---
bool g_is_waving = false;

// --- HELPER FUNCTION ---
void performWaveAction() {
    std::cout << "\n\033[1;35m>>> WAVE DETECTED! Greeting human... \033[0m" << std::endl;
    
    // Play Sound (Optional)
    // LinuxActionScript::PlayMP3Wait("/home/darwin/darwin/Data/mp3/demonstration_intro.mp3");

    if (Action::GetInstance()->IsRunning() == 0) {
        Action::GetInstance()->m_Joint.SetEnableBody(true, true);
        
        // Execute Wave
        Action::GetInstance()->Start(ACTION_PAGE_WAVE);
        
        // Wait for it to finish
        while (Action::GetInstance()->IsRunning()) {
            usleep(50000); 
        }
    }
}

int main(int argc, char* argv[]) {
    std::cout << "\n\033[1;36m=== Darwin-OP Gesture Interaction (Safe Mode) ===\033[0m" << std::endl;

    // 1. Initialize Motion Manager
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    // 2. Load Motion File
    if (Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH) == false) {
        std::cerr << "ERROR: Failed to load Motion file!" << std::endl;
        return 0;
    }

    // 3. Start Head Tracking Thread (It handles Python & Camera internally)
    std::cout << "INFO: Launching Vision Thread..." << std::endl;
    pthread_t tracking_thread;
    if (pthread_create(&tracking_thread, NULL, HeadTracking::AutoTrackingLoop, NULL) != 0) {
        std::cerr << "ERROR: Failed to create HeadTracking thread" << std::endl;
        return -1;
    }

    // 4. Robot Stand Up
    std::cout << "INFO: Robot Standing Up..." << std::endl;
    MotionManager::GetInstance()->SetEnable(true);
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    Action::GetInstance()->Start(ACTION_PAGE_READY); 
    while (Action::GetInstance()->IsRunning()) usleep(8000);

    std::cout << "\033[1;32mINFO: Ready! Waiting for wave...\033[0m" << std::endl;

    // 5. Main Loop
    int wave_counter = 0;

    while (true) {
        // Safe access via our new Getter
        std::string label = HeadTracking::GetInstance()->GetDetectedLabel();

        if (label == "hand_wave") {
            wave_counter++;
            std::cout << "DEBUG: Wave Validating... " << wave_counter << "/" << DETECT_THRESHOLD << "\r" << std::flush;

            if (wave_counter >= DETECT_THRESHOLD) {
                performWaveAction();
                
                // Reset and return to ready
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