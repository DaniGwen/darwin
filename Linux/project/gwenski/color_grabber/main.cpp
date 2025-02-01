/*
 * main.cpp - Ball Tracking & Grabbing Program
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <algorithm>
#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#define INI_FILE_PATH       "../../../../Data/config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"
#define GRAB_THRESHOLD      30
#define RELEASE_THRESHOLD   50
#define WRIST_GRAB_POS      700
#define GRIPPER_CLOSE_POS   800
#define WRIST_HOME_POS      512
#define GRIPPER_OPEN_POS    512

using namespace Robot;

// Arm position configurations
const int ARM_GRAB_POSITIONS[JointData::NUMBER_OF_JOINTS] = {
    0,  // [0] Unused
    // Right Arm              Left Arm
    512, 512,                // ID 1-2: Shoulder Pitch
    820, 204,                // ID 3-4: Shoulder Roll 
    512, 512,                // ID 5-6: Elbow
    // Hips, Knees, Ankles, Head
    512, 512, 512, 512, 512, 512, 512, 512, 512, 512,
    512, 512, 512, 512,      // ID 17-20
    WRIST_GRAB_POS,          // ID 21: Wrist
    GRIPPER_CLOSE_POS        // ID 22: Gripper
};

const int ARM_HOME_POSITIONS[JointData::NUMBER_OF_JOINTS] = {
    0,  // [0] Unused
    // Right Arm              Left Arm  
    512, 512,                // ID 1-2: Shoulder Pitch
    512, 512,                // ID 3-4: Shoulder Roll
    512, 512,                // ID 5-6: Elbow
    // Hips, Knees, Ankles, Head
    512, 512, 512, 512, 512, 512, 512, 512, 512, 512,
    512, 512, 512, 512,      // ID 17-20
    WRIST_HOME_POS,          // ID 21: Wrist
    GRIPPER_OPEN_POS         // ID 22: Gripper
};

void change_current_dir();
void set_arm_positions(CM730 &cm730, const int positions[], bool force = false);
void grab_ball(CM730 &cm730);
void release_arms(CM730 &cm730);

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    LinuxActionScript::PlayMP3("../../../Data/mp3/voice-to-battle.mp3");
    printf("\n===== Ball Tracking & Grabbing Program =====\n\n");
    change_current_dir();

    Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    minIni* ini = new minIni(INI_FILE_PATH);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* ball_finder = new ColorFinder();
    ball_finder->LoadINISettings(ini);
    httpd::ball_finder = ball_finder;

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();
    follower.DEBUG_PRINT = true;

    // Framework initialization
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);
    if(MotionManager::GetInstance()->Initialize(&cm730) == false) {
        printf("Fail to initialize Motion Manager!\n");
        return 0;
    }

    MotionManager::GetInstance()->LoadINISettings(ini);
    Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    // Initialize arm positions
    set_arm_positions(cm730, ARM_HOME_POSITIONS, true);
    usleep(1000000);

    printf("Press the ENTER key to begin!\n");
    getchar();
    
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    MotionManager::GetInstance()->SetEnable(true);

    bool isGrabbing = false;
    int grabTimeout = 0;

    while(1) {
        Point2D pos;
        LinuxCamera::GetInstance()->CaptureFrame();
        memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, 
               LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

        tracker.Process(ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));
        follower.Process(tracker.ball_position);

        // Grabbing logic
        if(tracker.ball_position.X != -1 && tracker.ball_position.Y != -1) {
            int centerX = Camera::WIDTH / 2;
            int centerY = Camera::HEIGHT / 2;
            int offsetX = abs(tracker.ball_position.X - centerX);
            int offsetY = abs(tracker.ball_position.Y - centerY);

            if(!isGrabbing && offsetX < GRAB_THRESHOLD && offsetY < GRAB_THRESHOLD) {
                grab_ball(cm730);
                isGrabbing = true;
                grabTimeout = 0;
            }
            else if(isGrabbing && (offsetX > RELEASE_THRESHOLD || offsetY > RELEASE_THRESHOLD)) {
                release_arms(cm730);
                isGrabbing = false;
            }
        }
        else if(isGrabbing) {
            if(++grabTimeout > 50) {
                release_arms(cm730);
                isGrabbing = false;
            }
        }

        // Draw detected ball area
        for(int i = 0; i < rgb_ball->m_NumberOfPixels; i++) {
            if(ball_finder->m_result->m_ImageData[i] == 1) {
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 0;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
            }
        }
        streamer->send_image(rgb_ball);
    }

    return 0;
}

void grab_ball(CM730 &cm730) {
    printf("**** GRABBING SEQUENCE INITIATED ****\n");
    Walking::GetInstance()->Stop();
    
    // Step 1: Position arm and wrist
    set_arm_positions(cm730, ARM_GRAB_POSITIONS);
    usleep(400000);
    
    // Step 2: Gradual gripper closure
    for(int pos = GRIPPER_OPEN_POS; pos <= GRIPPER_CLOSE_POS; pos += 20) {
        int target[JointData::NUMBER_OF_JOINTS];
        memcpy(target, ARM_GRAB_POSITIONS, sizeof(target));
        target[JointData::ID_R_GRIPPER] = pos;
        set_arm_positions(cm730, target);
        usleep(50000);
    }
    
    // Step 3: Final position adjustment
    set_arm_positions(cm730, ARM_GRAB_POSITIONS, true);
    usleep(500000);
}

void release_arms(CM730 &cm730) {
    printf("**** RELEASE SEQUENCE INITIATED ****\n");
    
    // Step 1: Open gripper
    int target[JointData::NUMBER_OF_JOINTS];
    memcpy(target, ARM_GRAB_POSITIONS, sizeof(target));
    target[JointData::ID_R_GRIPPER] = GRIPPER_OPEN_POS;
    set_arm_positions(cm730, target);
    usleep(500000);
    
    // Step 2: Return to home position
    set_arm_positions(cm730, ARM_HOME_POSITIONS);
    usleep(800000);
}

void set_arm_positions(CM730 &cm730, const int positions[], bool force) {
    int param[JointData::NUMBER_OF_JOINTS * 5];
    int n = 0;

    const int arm_joints[] = {
        JointData::ID_R_SHOULDER_PITCH,
        JointData::ID_R_SHOULDER_ROLL,
        JointData::ID_R_ELBOW,
        JointData::ID_R_WRIST,
        JointData::ID_R_GRIPPER
    };

    for(int i = 0; i < sizeof(arm_joints)/sizeof(int); i++) {
        int id = arm_joints[i];
        int current = MotionStatus::m_CurrentJoints.GetValue(id);
        int target = positions[id];

        if(!force && abs(current - target) < 10)
            continue;

        int speed = 100 + abs(current - target)/2;
        speed = std::min(speed, 200);

        param[n++] = id;
        param[n++] = CM730::GetLowByte(target);
        param[n++] = CM730::GetHighByte(target);
        param[n++] = CM730::GetLowByte(speed);
        param[n++] = CM730::GetHighByte(speed);
    }

    if(n > 0)
        cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, n/5, param);
}