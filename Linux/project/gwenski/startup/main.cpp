#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#define INI_FILE_PATH "config.ini"
#define U2D_DEV_NAME "/dev/ttyUSB0"

int main()
{
    printf("\n===== Start up with head tracking=====\n\n");

    minIni *ini = new minIni(INI_FILE_PATH);
    Image *rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    ColorFinder *red_ball_finder = new ColorFinder();
    red_ball_finder->LoadINISettings(ini);
    httpd::ball_finder = red_ball_finder;

    ColorFinder *blue_ball_finder = new ColorFinder();
    blue_ball_finder->LoadINISettings(ini, "Blue_Ball");

    BallTracker tracker = BallTracker();

    //////////////////// Framework Initialize ////////////////////////////
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);
    if (MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        printf("Fail to initialize Motion Manager!\n");
        LinuxActionScript::PlayMP3("../../../../Data/mp3/girl-scream.mp3");
        return 0;
    }
    MotionManager::GetInstance()->LoadINISettings(ini);
    MotionManager::GetInstance()->AddModule((MotionModule *)Head::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
    MotionManager::GetInstance()->SetEnable(true);
    /////////////////////////////////////////////////////////////////////

    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);

    Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 8);
    Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 8);

    LinuxActionScript::PlayMP3("../../../../Data/mp3/lock-and-load-male.mp3");

    while (1)
    {
        LinuxCamera::GetInstance()->CaptureFrame();

        tracker.Process(red_ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));
        tracker.Process(blue_ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));

        bool red_found = false;
        bool blue_found = false;
        rgb_ball = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;
        for (int i = 0; i < rgb_ball->m_NumberOfPixels; i++)
        {
            if (red_ball_finder->m_result->m_ImageData[i] == 1)
            {
                rgb_ball->m_ImageData[i * rgb_ball->m_PixelSize + 0] = 255;
                rgb_ball->m_ImageData[i * rgb_ball->m_PixelSize + 1] = 0;
                rgb_ball->m_ImageData[i * rgb_ball->m_PixelSize + 2] = 0;
                red_found = true;
            }
            if (blue_ball_finder->m_result->m_ImageData[i] == 1)
            {
                rgb_ball->m_ImageData[i * rgb_ball->m_PixelSize + 0] = 0;
                rgb_ball->m_ImageData[i * rgb_ball->m_PixelSize + 1] = 0;
                rgb_ball->m_ImageData[i * rgb_ball->m_PixelSize + 2] = 255;
                blue_found = true;
            }
        }

        // Set LED eyes to red if the red ball is found, blue if the blue ball is found
        if (red_found)
        {
            cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(255, 0, 0), 0);
        }
        else if (blue_found)
        {
            cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(0, 0, 255), 0);
        }
        else
        {
            cm730.WriteWord(CM730::ID_CM, CM730::P_LED_EYE_L, cm730.MakeColor(255, 255, 255), 0);
        }
    }

    return 0;
}
