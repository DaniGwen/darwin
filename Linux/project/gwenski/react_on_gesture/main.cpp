/*
 * main.cpp
 *
 * Mode: GESTURE INTERACTION
 * Description: 
 * - Dedicated script for Human-Robot Interaction via Gestures.
 * - Listens for "hand_wave" from the HeadTracking module.
 * - Triggers a greeting (Action Page 15) when waved at.
 */

#include "ConsoleColors.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <pthread.h>
#include <string>
#include <chrono>
#include <thread>

#include "HeadTracking.h"
#include "LinuxDARwIn.h"
#include "LinuxActionScript.h"

// --- Configuration ---
#define ACTION_PAGE_WAVE 15      // Standard Hello/Wave
#define ACTION_PAGE_STANDBY 2    // Standard Sit/Standby
#define ACTION_PAGE_READY 9      // Walk Ready / Standing

using namespace Robot;

// --- Tuning Constants ---
const int DETECT_THRESHOLD = 2;  // Frames of consistent detection required
const int WAVE_COOLDOWN_SEC = 10; // Seconds to wait after waving before looking again

// --- Global State ---
int wave_counter = 0;
bool is_interacting = false;

// --- Helper Functions ---

void performWaveAction() {
    std::cout << Color::FG_MAGENTA << ">>> 👋 WAVE DETECTED! Greeting human..." << Color::FG_DEFAULT << std::endl;

    // 1. Play Sound (Optional - ensure file exists)
    // LinuxActionScript::PlayMP3Wait("/home/darwin/darwin/Data/mp3/hello.mp3");

    // 2. Execute Motion
    if (Action::GetInstance()->IsRunning() == 0) {
        // Ensure body is enabled
        Action::GetInstance()->m_Joint.SetEnableBody(true, true);
        
        // Start the Wave (Page 15)
        Action::GetInstance()->Start(ACTION_PAGE_WAVE);
        
        // Wait for the action to finish so we don't interrupt it
        while (Action::GetInstance()->IsRunning()) {
            usleep(50000); // Check every 50ms
        }
    }
}

int main(int argc, char* argv[]) {
    std::cout << Color::FG_CYAN << "=== Darwin-OP Gesture Interaction Mode ===" << Color::FG_DEFAULT << std::endl;

    // 1. Initialize Motion Framework
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    // 2. Load Action File
    if (Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH) == false) {
        std::cerr << "ERROR: Failed to load Motion file!" << std::endl;
        return 0;
    }

    // 3. Start Head Tracking Thread
    // IMPORTANT: Ensure HeadTracking.cpp is pointing to 'gesture_detector.py'
    std::cout << "INFO: Launching Vision Thread..." << std::endl;
    pthread_t tracking_thread;
    if (pthread_create(&tracking_thread, NULL, HeadTracking::AutoTrackingLoop, NULL) != 0) {
        std::cerr << "ERROR: Failed to create thread" << std::endl;
        return -1;
    }

    // 4. Robot Startup Sequence
    std::cout << "INFO: Robot Standing Up..." << std::endl;
    MotionManager::GetInstance()->SetEnable(true);
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    Action::GetInstance()->Start(ACTION_PAGE_READY); 
    while (Action::GetInstance()->IsRunning()) usleep(8000);

    std::cout << Color::FG_GREEN << "INFO: Ready! Waiting for user to wave..." << Color::FG_DEFAULT << std::endl;

    // 5. Main Interaction Loop
    while (true) {
        // Get the latest label from the Vision Thread
        std::string label = HeadTracking::GetInstance()->present_detected_object_label;

        // --- Logic: Detect Wave ---
        if (label == "hand_wave") {
            wave_counter++;
            std::cout << "DEBUG: Wave Validating... " << wave_counter << "/" << DETECT_THRESHOLD << "\r" << std::flush;

            if (wave_counter >= DETECT_THRESHOLD) {
                // Confirmed!
                performWaveAction();

                // Reset state
                wave_counter = 0;
                HeadTracking::GetInstance()->current_detected_label_ = "none";
                
                // Return to Ready pose
                Action::GetInstance()->Start(ACTION_PAGE_READY);
                while (Action::GetInstance()->IsRunning()) usleep(8000);
                
                std::cout << "INFO: Ready for next gesture." << std::endl;
            }
        } 
        else {
            // If we lose the detection, decay the counter quickly
            if (wave_counter > 0) wave_counter--;
        }

        // Small delay to prevent CPU overuse
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Cleanup (Unlikely to reach here in infinite loop)
    pthread_join(tracking_thread, NULL);
    motion_timer->Stop();
    MotionManager::GetInstance()->SetEnable(false);
    return 0;
}