#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>

#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH "../../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH "../../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH "config.ini"
#define SCRIPT_FILE_PATH "script.asc"

#define U2D_DEV_NAME0 "/dev/ttyUSB0"
#define U2D_DEV_NAME1 "/dev/ttyUSB1"

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

struct ServoData
{
    int Id;
    int Position;
};

static ServoData rigth_arm_data_ready[5] = {
    {JointData::ID_R_SHOULDER_PITCH, 1324},
    {JointData::ID_R_SHOULDER_ROLL, 2025},
    {JointData::ID_R_ELBOW, 1985},
    {JointData::ID_R_WRIST, 2424},
    {JointData::ID_R_GRIPPER, 1451}};

static ServoData rigth_arm_data_pickup[5] = {
    {JointData::ID_R_SHOULDER_PITCH, 1714},
    {JointData::ID_R_SHOULDER_ROLL, 1868},
    {JointData::ID_R_ELBOW, 1525},
    {JointData::ID_R_WRIST, 2317},
    {JointData::ID_R_GRIPPER, 1451}};

bool WaitWhileServoMoving(CM730 &cm730, int servo_id)
{
    int moving_status;
    int timeout = 100; // 100 * 10ms = 1 second timeout
    while (
        cm730.ReadByte(servo_id, MX28::P_MOVING, &moving_status, 0) == CM730::SUCCESS &&
        moving_status == 1 &&
        timeout-- > 0)
    {
        usleep(2 * 10000);
    }

    if (timeout <= 0)
    {
        printf("\nTimeout: Servo did not reach goal!\n");
        return false;
    }
    return true;
}

int main(void)
{
    minIni *ini = new minIni(INI_FILE_PATH);
    Image *rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings()); // set default
    LinuxCamera::GetInstance()->LoadINISettings(ini);                // load from ini

    mjpg_streamer *streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder *ball_finder = new ColorFinder();
    ball_finder->LoadINISettings(ini);
    httpd::ball_finder = ball_finder;

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();
    follower.DEBUG_PRINT = true;

    ColorFinder *red_finder = new ColorFinder(0, 15, 45, 0, 0.3, 50.0);
    red_finder->LoadINISettings(ini, "RED");
    httpd::red_finder = red_finder;

    ColorFinder *yellow_finder = new ColorFinder(60, 15, 45, 0, 0.3, 50.0);
    yellow_finder->LoadINISettings(ini, "YELLOW");
    httpd::yellow_finder = yellow_finder;

    ColorFinder *blue_finder = new ColorFinder(225, 15, 45, 0, 0.3, 50.0);
    blue_finder->LoadINISettings(ini, "BLUE");
    httpd::blue_finder = blue_finder;

    httpd::ini = ini;

    //////////////////// Framework Initialize ////////////////////////////
    if (MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if (MotionManager::GetInstance()->Initialize(&cm730) == false)
        {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
        }
    }

    Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule *)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule *)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule *)Walking::GetInstance());

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    /////////////////////////////////////////////////////////////////////

    MotionManager::GetInstance()->LoadINISettings(ini);

    int firm_ver = 0;
    if (cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0) != CM730::SUCCESS)
    {
        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
        exit(0);
    }

    if (0 < firm_ver && firm_ver < 27)
    {
#ifdef MX28_1024
        Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
#else
        fprintf(stderr, "MX-28's firmware is not support 4096 resolution!! \n");
        fprintf(stderr, "Upgrade MX-28's firmware to version 27(0x1B) or higher.\n\n");
        exit(0);
#endif
    }
    else if (27 <= firm_ver)
    {
#ifdef MX28_1024
        fprintf(stderr, "MX-28's firmware is not support 1024 resolution!! \n");
        fprintf(stderr, "Remove '#define MX28_1024' from 'MX28.h' file and rebuild.\n\n");
        exit(0);
#else
        Action::GetInstance()->LoadFile((char *)MOTION_FILE_PATH);
#endif
    }
    else
        exit(0);

    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    MotionManager::GetInstance()->SetEnable(true);

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01 | 0x02 | 0x04, NULL);

    LinuxActionScript::PlayMP3Wait("../../../../Data/mp3/activation-finished.mp3");
    Action::GetInstance()->Start(16);
    printf("Standing up...\n");
    while (Action::GetInstance()->IsRunning())
        usleep(8 * 1000);

    int ball_found = 0;

    Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 5);
    Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 5);

    while (1)
    {
        Point2D pos;
        LinuxCamera::GetInstance()->CaptureFrame();

        memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

        tracker.Process(ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));
        follower.Process(tracker.ball_position);

        for (int i = 0; i < rgb_output->m_NumberOfPixels; i++)
        {
            if (ball_finder->m_result->m_ImageData[i] == 1)
            {
                rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 0] = 255;
                rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 1] = 128;
                rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 2] = 0;
            }
        }

        streamer->send_image(rgb_output);

        if (Action::GetInstance()->IsRunning() == 0)
        {
            Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
            Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

            if (follower.KickBall != 0)
            {
                LinuxActionScript::PlayMP3Wait("../../../../Data/mp3/target-acquired.mp3");

                Walking::GetInstance()->A_MOVE_AMPLITUDE = -10; // turn left
                usleep(20 * 1000);

                Walking::GetInstance()->Stop();

                Action::GetInstance()->Start(15); // sit down
                while (Action::GetInstance()->IsRunning())
                    usleep(8 * 1000);

                int number_of_joints = sizeof(rigth_arm_data_ready) / sizeof(rigth_arm_data_ready[0]);
                for (int i = 0; i < number_of_joints; i++)
                {
                    cm730.WriteByte(rigth_arm_data_ready[i].Id, MX28::P_P_GAIN, 6, 0);
                    cm730.WriteWord(rigth_arm_data_ready[i].Id, MX28::P_GOAL_POSITION_L, rigth_arm_data_ready[i].Position, 0);
                    WaitWhileServoMoving(cm730, rigth_arm_data_ready[i].Id);
                }

                if (follower.KickBall == -1) // right side
                {
                    cm730.WriteByte(JointData::ID_R_GRIPPER, MX28::P_P_GAIN, 8, 0);

                    for (int i = 0; i < number_of_joints; i++)
                    {
                        cm730.WriteWord(rigth_arm_data_pickup[i].Id, MX28::P_GOAL_POSITION_L, rigth_arm_data_ready[i].Position, 0);
                        WaitWhileServoMoving(cm730, rigth_arm_data_pickup[i].Id);
                    }

                    fprintf(stderr, "picking up... \n");
                }
                else if (follower.KickBall == 1) // left side
                {
                    fprintf(stderr, "object left side\n");
                }

                Action::GetInstance()->Start(16); // stand up
                while (Action::GetInstance()->IsRunning())
                    usleep(8 * 1000);
            }
        }
    }

    return 0;
}
