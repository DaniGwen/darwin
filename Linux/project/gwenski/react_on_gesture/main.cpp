/*
 * main.cpp
 * FIXED VERSION 2.0
 * - Fixes compilation errors (LinuxCamera/FrameBuffer access).
 * - Uses correct Image structure access.
 * - Runs Vision Thread locally to talk to Python.
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

// --- FRAMEWORK HEADERS ---
#include "HeadTracking.h"
#include "LinuxDARwIn.h"
#include "LinuxActionScript.h"

// --- TRICK: Access protected members of LinuxCamera ---
#define protected public
#include "LinuxCamera.h"
#undef protected

using namespace Robot;

// --- DEFINITIONS ---
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#define MOTION_FILE_PATH   "../../../../Data/motion_4096.bin"
#define ACTION_PAGE_WAVE   15
#define ACTION_PAGE_READY  9
#define PYTHON_SCRIPT      "/home/darwin/darwin/aiy-maker-kit/examples/gesture_detector.py"
#define SOCKET_PATH        "/tmp/darwin_detector.sock"

// --- GLOBAL VARIABLES ---
std::string g_detected_label = "none";
pthread_mutex_t g_label_mutex = PTHREAD_MUTEX_INITIALIZER;
bool g_running = true;

// --- VISION THREAD ---
void* VisionThreadLoop(void* arg) {
    std::cout << "[Vision] Starting Vision Thread..." << std::endl;

    // 1. Launch Python Script
    std::string cmd = "python3 " + std::string(PYTHON_SCRIPT) + " &";
    system(cmd.c_str());
    sleep(3); // Wait for Python to load

    // 2. Create Socket Server
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

    unlink(SOCKET_PATH); 
    if (bind(server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("[Vision] Bind error");
        return NULL;
    }

    listen(server_sock, 1);
    std::cout << "[Vision] Waiting for Python connection..." << std::endl;
    
    // 3. Accept Connection
    client_sock = accept(server_sock, NULL, NULL);
    if (client_sock < 0) {
        perror("[Vision] Accept error");
        return NULL;
    }
    std::cout << "[Vision] Connected!" << std::endl;

    // 4. Initialize Camera
    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(LinuxCamera::GetInstance()->GetCameraSettings());

    // 5. Main Loop
    while (g_running) {
        // Capture Frame
        LinuxCamera::GetInstance()->CaptureFrame();
        
        // --- FIXED ACCESS CODE ---
        // Access the protected 'fbuffer' directly due to our #define hack
        FrameBuffer* fb = LinuxCamera::GetInstance()->fbuffer;
        
        // Use the RGB Image object
        Image* img = fb->m_RGBFrame; 
        
        int width = img->m_Width;
        int height = img->m_Height;
        unsigned char* data = img->m_ImageData;
        // -------------------------

        // Send Header
        send(client_sock, &width, sizeof(int), 0);
        send(client_sock, &height, sizeof(int), 0);

        // Send Data
        int data_size = width * height * 3;
        send(client_sock, data, data_size, 0);

        // Receive Response
        unsigned int msg_len = 0;
        int n = recv(client_sock, &msg_len, sizeof(msg_len), 0);
        if (n <= 0) break;

        if (msg_len > 0) {
            std::vector<char> buffer(msg_len + 1);
            recv(client_sock, buffer.data(), msg_len, 0);
            buffer[msg_len] = '\0';
            
            std::string response(buffer.data());
            
            // Extract label (first word)
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

// --- HELPER FUNCTIONS ---
void performWaveAction() {
    std::cout << ANSI_COLOR_MAGENTA << ">>> 👋 WAVE DETECTED! Greeting human..." << ANSI_COLOR_RESET << std::endl;
    
    if (Action::GetInstance()->IsRunning() == 0) {
        Action::GetInstance()->m_Joint.SetEnableBody(true, true);
        Action::GetInstance()->Start(ACTION_PAGE_WAVE);
        
        // Wait for action to complete
        while (Action::GetInstance()->IsRunning()) {
            usleep(50000); 
        }
    }
}

// --- MAIN ---
int main(int argc, char* argv[]) {
    std::cout << ANSI_COLOR_CYAN << "=== Darwin-OP Gesture Interaction ===" << ANSI_COLOR_RESET << std::endl;

    // Setup Motion
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    if (Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH) == false) {
        std::cerr << ANSI_COLOR_RED << "ERROR: Failed to load Motion file!" << ANSI_COLOR_RESET << std::endl;
        return 0;
    }

    // Launch Vision
    pthread_t vision_thread;
    if (pthread_create(&vision_thread, NULL, VisionThreadLoop, NULL) != 0) {
        std::cerr << "ERROR: Failed to create vision thread" << std::endl;
        return -1;
    }

    // Start Robot
    MotionManager::GetInstance()->SetEnable(true);
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    Action::GetInstance()->Start(ACTION_PAGE_READY);
    while (Action::GetInstance()->IsRunning()) usleep(8000);

    std::cout << ANSI_COLOR_GREEN << "INFO: Ready! Waiting for wave..." << ANSI_COLOR_RESET << std::endl;

    int wave_counter = 0;
    const int DETECT_THRESHOLD = 2;

    while (true) {
        pthread_mutex_lock(&g_label_mutex);
        std::string label = g_detected_label;
        pthread_mutex_unlock(&g_label_mutex);

        if (label == "hand_wave") {
            wave_counter++;
            std::cout << "DEBUG: Wave Validating... " << wave_counter << "/" << DETECT_THRESHOLD << "\r" << std::flush;

            if (wave_counter >= DETECT_THRESHOLD) {
                performWaveAction();
                wave_counter = 0;
                
                // Return to ready
                Action::GetInstance()->Start(ACTION_PAGE_READY);
                while (Action::GetInstance()->IsRunning()) usleep(8000);
            }
        } else {
            if (wave_counter > 0) wave_counter--;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    g_running = false;
    pthread_join(vision_thread, NULL);
    motion_timer->Stop();
    MotionManager::GetInstance()->SetEnable(false);
    return 0;
}