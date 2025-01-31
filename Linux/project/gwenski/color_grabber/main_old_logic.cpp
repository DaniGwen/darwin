#include <cmath>
#include "GripperControl.h"
#include "LinuxDARwIn.h"
#include <stdio.h>
#include <unistd.h>

// Color detection thresholds (adjust these based on your environment)
#define RED_HUE 0
#define YELLOW_HUE 60
#define BLUE_HUE 225
#define COLOR_MARGIN 15
#define COLOR_SAT 45
#define COLOR_VALUE 50

#define INI_FILE_PATH          "config.ini"
#define INI_FILE_PATH_MOTION   "../../../../Data/config.ini"

// Movement parameters
#define APPROACH_SPEED -0.5
#define TURN_GAIN 0.3

// Object size thresholds
#define MIN_OBJECT_AREA 50
#define MAX_OBJECT_AREA 5000

ColorFinder *color_finder = NULL;
Point2D target_position;

void InitializeVision()
{
    minIni *ini = new minIni(INI_FILE_PATH);
    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    // Create color finder with parameters for binary thresholding
    color_finder = new ColorFinder(
        RED_HUE,      // Initial hue (will be changed)
        COLOR_MARGIN, // Hue margin
        COLOR_SAT,    // Minimum saturation
        COLOR_VALUE,  // Minimum value
        0.3,          // Minimum probability
        50.0          // Distance penalty
    );
    color_finder->LoadINISettings(ini);
}

double CalculateObjectArea(Image *result_image)
{
    if (!result_image || result_image->m_PixelSize != 1)
        return 0.0;

    int count = 0;
    for (int i = 0; i < result_image->m_NumberOfPixels; i++)
    {
        if (result_image->m_ImageData[i] == 1)
            count++;
    }
    return static_cast<double>(count);
}

bool LocateColoredObject(CM730 &cm730, int &detected_color)
{
    LinuxCamera::GetInstance()->CaptureFrame();
    Image *hsv_frame = LinuxCamera::GetInstance()->fbuffer->m_HSVFrame;

    // Configure color finder for red detection
    color_finder->m_hue = RED_HUE;
    Point2D red_pos = color_finder->GetPosition(hsv_frame);
    double red_area = CalculateObjectArea(color_finder->m_result);

    // Configure color finder for yellow detection
    color_finder->m_hue = YELLOW_HUE;
    Point2D yellow_pos = color_finder->GetPosition(hsv_frame);
    double yellow_area = CalculateObjectArea(color_finder->m_result);

    // Configure color finder for blue detection
    color_finder->m_hue = BLUE_HUE;
    Point2D blue_pos = color_finder->GetPosition(hsv_frame);
    double blue_area = CalculateObjectArea(color_finder->m_result);

    // Find maximum area
    double max_area = 0.0;
    detected_color = 0;

    if (red_area > max_area && red_area > MIN_OBJECT_AREA)
    {
        max_area = red_area;
        detected_color = 1;
        target_position = red_pos; // Use already captured position
    }

    if (yellow_area > max_area && yellow_area > MIN_OBJECT_AREA)
    {
        max_area = yellow_area;
        detected_color = 2;
        target_position = yellow_pos; // Use already captured position
    }

    if (blue_area > max_area && blue_area > MIN_OBJECT_AREA)
    {
        max_area = blue_area;
        detected_color = 3;
        target_position = blue_pos; // Use already captured position
    }

    return (max_area > MIN_OBJECT_AREA && max_area < MAX_OBJECT_AREA);
}

void AlignWithObject(CM730 &cm730, int &detected_color)
{
    const int center_x = Camera::WIDTH / 2;
    const int margin = 20;

    while (fabs(target_position.X - center_x) > margin)
    {
        double error = (target_position.X - center_x) / center_x;

        Walking::GetInstance()->X_MOVE_AMPLITUDE = 0.0;
        Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0.0;
        Walking::GetInstance()->A_MOVE_AMPLITUDE = TURN_GAIN * error;
        Walking::GetInstance()->Start();

        usleep(10000);
        if (!LocateColoredObject(cm730, detected_color))
            return;
    }
    Walking::GetInstance()->Stop();
}

void ApproachObject(CM730 &cm730)
{
    PositionRightArm(cm730);
    CenterGripperPosition(cm730);

    // Lower elbow to reach position
    cm730.WriteWord(JointData::ID_R_ELBOW, MX28::P_GOAL_POSITION_L, 2500, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_ELBOW);

    // Approach while maintaining alignment
    for (int i = 0; i < 20; i++)
    {
        Walking::GetInstance()->X_MOVE_AMPLITUDE = APPROACH_SPEED;
        Walking::GetInstance()->Start();
        usleep(50000);
    }
    Walking::GetInstance()->Stop();
}

void GrabSequence(CM730 &cm730, int &detected_color)
{
    // Final precise positioning
    AlignWithObject(cm730, detected_color);

    // Close gripper
    CloseGripper(cm730);

    // Lift object
    cm730.WriteWord(JointData::ID_R_ELBOW, MX28::P_GOAL_POSITION_L, 2000, 0);
    WaitWhileServoMoving(cm730, JointData::ID_R_ELBOW);

    // Move back to safe position
    Walking::GetInstance()->X_MOVE_AMPLITUDE = -APPROACH_SPEED;
    Walking::GetInstance()->Start();
    usleep(1000000);
    Walking::GetInstance()->Stop();
}

int main()
{
    LinuxCM730 linux_cm730("/dev/ttyUSB0");
    CM730 cm730(&linux_cm730);
     minIni* ini = new minIni(INI_FILE_PATH_MOTION);

    // Initialize systems
    if (!cm730.Connect())
    {
        printf("Failed to connect CM-730!\n");
        return 1;
    }

    MotionManager::GetInstance()->Initialize(&cm730);
    MotionManager::GetInstance()->LoadINISettings(ini);
    Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule *)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule *)Walking::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    InitializeGripper(cm730);
    InitializeVision();

    // Main detection loop
    int detected_color = 0;
    while (1)
    {
        if (LocateColoredObject(cm730, detected_color))
        {
            printf("Detected %s object\n",
                   (detected_color == 1) ? "RED" : (detected_color == 2) ? "YELLOW"
                                                                         : "BLUE");

            AlignWithObject(cm730, detected_color);
            ApproachObject(cm730);
            GrabSequence(cm730, detected_color);

            // Release object
            OpenGripper(cm730);
            PositionRightArm(cm730);

            printf("Object handling complete!\n");
            sleep(2);
        }
        else
        {
            // Search pattern
            Head::GetInstance()->MoveByAngle(0, 30);
            usleep(500000);
            Head::GetInstance()->MoveByAngle(0, -30);
            usleep(500000);
        }
    }

    return 0;
}