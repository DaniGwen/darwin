/*
 * main.cpp
 * FIXED VERSION
 * - Defines Colors and Paths locally (Fixes "Color not declared" / "MOTION_FILE_PATH" error)
 * - Implements a local Vision Thread (Fixes "AutoTrackingLoop" / "private member" errors)
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
#include <vector>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>

#include "HeadTracking.h"
#include "LinuxDARwIn.h"
#include "LinuxActionScript.h"
#include "LinuxCamera.h"

using namespace Robot;

// --- 1. DEFINITIONS (Fixing Missing Headers) ---
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#define MOTION_FILE_PATH   "../../../../Data/motion_4096.bin"
#define ACTION_PAGE_WAVE   15
#define ACTION_PAGE_READY  9
#define PYTHON_SCRIPT      "/home/darwin/darwin/aiy-maker-kit/examples/gesture_detector.py"
#define SOCKET_PATH        "/tmp/darwin_detector.sock"

// --- 2. GLOBAL VARIABLES ---
std::string g_detected_label = "none";
pthread_mutex_t g_label_mutex = PTHREAD_MUTEX_INITIALIZER;
bool g_running = true;

// --- 3. VISION THREAD (Replaces HeadTracking logic) ---
void* VisionThreadLoop(void* arg) {
    std::cout << ANSI_COLOR_BLUE << "[Vision] Starting Vision Thread..." << ANSI_COLOR_RESET << std::endl;

    // A. Launch Python Script in background
    std::string cmd = "python3 " + std::string(PYTHON_SCRIPT) + " &";
    system(cmd.c_str());
    sleep(2); // Give Python time to start

    // B. Create Socket Server
    int server_sock, client_sock;
    struct sockaddr_un server_addr;

    server_sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_sock < 0) {
        perror("[Vision] Socket error");
        return NULL;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sun_family = AF_UNIX;
    strncpy(server_addr.sun_path, SOCKET_PATH, sizeof(server_addr.sun_path) - 1);

    unlink(SOCKET_PATH); // Remove old socket
    if (bind(server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("[Vision] Bind error");
        return NULL;
    }

    listen(server_sock, 1);
    std::cout << ANSI_COLOR_BLUE << "[Vision] Waiting for Python connection..." << ANSI_COLOR_RESET << std::endl;
    
    // C. Accept Connection
    client_sock = accept(server_sock, NULL, NULL);
    if (client_sock < 0) {
        perror("[Vision] Accept error");
        return NULL;
    }
    std::cout << ANSI_COLOR_GREEN << "[Vision] Connected!" << ANSI_COLOR_RESET << std::endl;

    // D. Initialize Camera
    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(LinuxCamera::GetInstance()->GetCameraSettings());

    // E. Main Vision Loop
    while (g_running) {
        // Capture Frame
        LinuxCamera::GetInstance()->CaptureFrame();
        FrameBuffer* fb = LinuxCamera::GetInstance()->GetFrameBuffer();
        
        // Send Header (Width, Height)
        int width = fb->m_GPixelFormat->m_Width;
        int height = fb->m_GPixelFormat->m_Height;
        send(client_sock, &width, sizeof(int), 0);
        send(client_sock, &height, sizeof(int), 0);

        // Send RGB Data
        // Note: RGB Frame buffer size is Width * Height * 3
        int data_size = width * height * 3;
        send(client_sock, fb->m_RGBFrame->m_ImageData, data_size, 0);

        // Receive Response (Length + String)
        unsigned int msg_len = 0;
        int n = recv(client_sock, &msg_len, sizeof(msg_len), 0);
        if (n <= 0) break;

        if (msg_len > 0) {
            std::vector<char> buffer(msg_len + 1);
            recv(client_sock, buffer.data(), msg_len, 0);
            buffer[msg_len] = '\0';
            
            std::string response(buffer.data());
            
            // Parse: "label score x y..."
            char label_buf[64];
            sscanf(response.c_str(), "%s", label_buf);
            
            pthread_mutex_lock(&g_label_mutex);
            g_detected_label = std::string(label_buf);
            pthread_mutex_unlock(&g_label_mutex);
        } else {
             pthread_mutex_lock(&g_label_mutex);
             g_detected_label = "none";
             pthread_mutex_unlock(&g_label_mutex);
        }
    }

    close(client_sock);
    close(server_sock);
    return NULL;
}

// --- 4. HELPER FUNCTIONS ---
void performWaveAction() {
    std::cout << ANSI_COLOR_MAGENTA << ">>> 👋 WAVE DETECTED! Greeting human..." << ANSI_COLOR_RESET << std::endl;

    // Optional: Play Sound
    // LinuxActionScript::PlayMP3Wait("/home/darwin/darwin/Data/mp3/hello.mp3");

    if (Action::GetInstance()->IsRunning() == 0) {
        Action::GetInstance()->m_Joint.SetEnableBody(true, true);
        Action::GetInstance()->Start(ACTION_PAGE_WAVE);
        
        // Wait for action to complete
        while (Action::GetInstance()->IsRunning()) {
            usleep(50000); 
        }
    }
}

// --- 5. MAIN ---
int main(int argc, char* argv[]) {
    std::cout << ANSI_COLOR_CYAN << "=== Darwin-OP Gesture Interaction Mode ===" << ANSI_COLOR_RESET << std::endl;

    // Initialize Motion
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    if (Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH) == false) {
        std::cerr << ANSI_COLOR_RED << "ERROR: Failed to load Motion file!" << ANSI_COLOR_RESET << std::endl;
        return 0;
    }

    // Launch LOCAL Vision Thread (Bypassing HeadTracking class issues)
    pthread_t vision_thread;
    if (pthread_create(&vision_thread, NULL, VisionThreadLoop, NULL) != 0) {
        std::cerr << "ERROR: Failed to create vision thread" << std::endl;
        return -1;
    }

    // Robot Startup
    MotionManager::GetInstance()->SetEnable(true);
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    Action::GetInstance()->Start(ACTION_PAGE_READY);
    while (Action::GetInstance()->IsRunning()) usleep(8000);

    std::cout << ANSI_COLOR_GREEN << "INFO: Ready! Waiting for user to wave..." << ANSI_COLOR_RESET << std::endl;

    // Main Interaction Loop
    int wave_counter = 0;
    const int DETECT_THRESHOLD = 2;

    while (true) {
        // Read Global Label safely
        pthread_mutex_lock(&g_label_mutex);
        std::string label = g_detected_label;
        pthread_mutex_unlock(&g_label_mutex);

        if (label == "hand_wave") {
            wave_counter++;
            std::cout << "DEBUG: Wave Validating... " << wave_counter << "/" << DETECT_THRESHOLD << "\r" << std::flush;

            if (wave_counter >= DETECT_THRESHOLD) {
                performWaveAction();
                
                // Reset
                wave_counter = 0;
                pthread_mutex_lock(&g_label_mutex);
                g_detected_label = "none";
                pthread_mutex_unlock(&g_label_mutex);

                // Return to Ready
                Action::GetInstance()->Start(ACTION_PAGE_READY);
                while (Action::GetInstance()->IsRunning()) usleep(8000);
                std::cout << "INFO: Ready for next gesture." << std::endl;
            }
        } else {
            if (wave_counter > 0) wave_counter--;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Cleanup
    g_running = false;
    pthread_join(vision_thread, NULL);
    motion_timer->Stop();
    MotionManager::GetInstance()->SetEnable(false);
    return 0;
}