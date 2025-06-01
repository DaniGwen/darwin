#include "RightArmController.h"
#include "ConsoleColors.h"
#include "cmath"
#include "HeadTracking.h" // Needed for Value2Deg
#include "Point.h"

namespace Robot
{

    RightArmController::RightArmController(CM730 *cm730)
        : cm730_(cm730),
          m_Hand_P_Gain_X(0.02),
          m_Hand_P_Gain_Y(0.02)
    {
        if (!cm730_)
        {
            std::cerr << "ERROR: RightArmController initialized with a NULL CM730 pointer. Motor control will not be possible." << std::endl;
        }
    }

    void RightArmController::ApplyPose(const Pose &pose, int speed)
    {
        if (!cm730_)
        {
            std::cerr << BOLDRED << "ERROR: CM730 not initialized in LegsController, cannot apply pose." << RESET << std::endl;
            return;
        }

        // Define PID gains and Moving Speed for this combined command
        const int DEFAULT_P_GAIN = 32;
        const int DEFAULT_I_GAIN = 0;
        const int DEFAULT_D_GAIN = 0;
        const int RESERVED_BYTE = 0;
        const int MOVING_SPEED = speed; // between (0-1023)

        // Total items per motor in the params array: ID + D + I + P + Res + PosL + PosH + SpeedL + SpeedH
        const int DATA_CHUNK_SIZE = 9;

        std::lock_guard<std::mutex> lock(cm730_mutex);

        std::vector<int> params;
        params.reserve(pose.joint_positions.size() * DATA_CHUNK_SIZE);

        for (const auto &joint_pair : pose.joint_positions)
        {
            int joint_id = joint_pair.first;
            int goal_value = joint_pair.second;

            params.push_back(joint_id);                         // Item 1: ID
            params.push_back(DEFAULT_D_GAIN);                   // Item 2: D Gain
            params.push_back(DEFAULT_I_GAIN);                   // Item 3: I Gain
            params.push_back(DEFAULT_P_GAIN);                   // Item 4: P Gain
            params.push_back(RESERVED_BYTE);                    // Item 5: Reserved
            params.push_back(CM730::GetLowByte(goal_value));    // Item 6: Goal Position Low
            params.push_back(CM730::GetHighByte(goal_value));   // Item 7: Goal Position High
            params.push_back(CM730::GetLowByte(MOVING_SPEED));  // Item 8: Moving Speed Low
            params.push_back(CM730::GetHighByte(MOVING_SPEED)); // Item 9: Moving Speed High
        }

        if (!params.empty())
        {
            std::cout << BOLDCYAN << "INFO: Applying combined PID, Pose, and Speed via SyncWrite..." << RESET << std::endl;

            int num_joints = params.size() / DATA_CHUNK_SIZE;

            // Start Address: D Gain. Length of chunk (incl. ID for your SyncWrite wrapper): 9.
            int result = cm730_->SyncWrite(MX28::P_D_GAIN, DATA_CHUNK_SIZE, num_joints, params.data());

            if (result == cm730_->SUCCESS)
            {
                std::cout << BOLDGREEN << "INFO: Combined pose applied successfully." << RESET << std::endl;
            }
            else
            {
                std::cerr << BOLDRED << "ERROR: Combined SyncWrite failed with code: " << result << RESET << std::endl;
            }
        }
    }

    void RightArmController::CloseGripper(int moving_speed, int p_gain)
    {
        SetPID(p_gain);

        std::cout << "INFO: Moving right arm to POSE_CLOSE_GRIPPER ..." << std::endl;
        ApplyPose(POSE_CLOSE_GRIPPER, moving_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    void RightArmController::OpenGripper(int moving_speed, int p_gain)
    {
        SetPID(p_gain);

        std::cout << "INFO: Moving right arm to POSE_OPEN_GRIPPER ..." << std::endl;
        ApplyPose(POSE_OPEN_GRIPPER, moving_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    void RightArmController::RotateWristCW90Deg(int moving_speed, int p_gain)
    {
        SetPID(p_gain);

        std::cout << "INFO: Moving right arm to POSE_ROTATE_WRIST_90DEG ..." << std::endl;
        ApplyPose(POSE_ROTATE_WRIST_90DEG, moving_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    void RightArmController::RotateWristCCW90Deg(int moving_speed, int p_gain)
    {
        SetPID(p_gain);

        std::cout << "INFO: Moving right arm to POSE_ROTATE_WRIST_90DEG ..." << std::endl;
        ApplyPose(POSE_ROTATE_WRIST_CCW_90DEG, moving_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    void RightArmController::HoldItem(int moving_speed, int p_gain)
    {
        SetPID(p_gain);

        std::cout << "INFO: Moving right arm to POSE_ROTATE_WRIST_90DEG ..." << std::endl;
        ApplyPose(POSE_HOLD_ITEM, moving_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }

    void RightArmController::Default()
    {
        SetPID();

        std::cout << "INFO: Resetting right arm to default pose..." << std::endl;
        ApplyPose(DEFAULT);
        std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    }

    Point2D RightArmController::GetHandPositionInCameraView()
    {
        if (!cm730_)
        {
            return Point2D(-1, -1); // Return invalid point
        }

        // --- Step 1: Read Current Joint Angles from Motors ---
        int shoulder_pitch_val, shoulder_roll_val, elbow_val;
        // Read the 16-bit "Present Position" value from each motor.
        cm730_->ReadWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &shoulder_pitch_val, 0);
        cm730_->ReadWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_PRESENT_POSITION_L, &shoulder_roll_val, 0);
        cm730_->ReadWord(JointData::ID_R_ELBOW, MX28::P_PRESENT_POSITION_L, &elbow_val, 0);

        // Convert raw values to degrees, then to radians for C++ math functions.
        // The Value2Deg function is a static method in the HeadTracking class.
        double shoulder_pitch_rad = Robot::HeadTracking::Value2Deg(shoulder_pitch_val) * (M_PI / 180.0);
        double shoulder_roll_rad = Robot::HeadTracking::Value2Deg(shoulder_roll_val) * (M_PI / 180.0);
        double elbow_rad = Robot::HeadTracking::Value2Deg(elbow_val) * (M_PI / 180.0);

        // --- Step 2: Forward Kinematics using Dimensions from Image ---
        // All dimensions are converted from mm to meters.
        // The origin (0,0,0) is at the robot's shoulder roll joint.

        // Length of the upper arm segment (shoulder roll to elbow).
        // From schematic: 129.0mm (torso center to elbow) - 60.0mm (torso center to shoulder) = 69.0mm
        const double ARM_UPPER_LENGTH_M = 0.069;

        // Length of the forearm segment (elbow to end-effector).
        // From schematic: 60.0mm from the elbow joint to the end of the arm block.
        const double ARM_LOWER_LENGTH_M = 0.060;

        // Simplified forward kinematics calculation.
        // Assumes X is forward, Y is to the left, and Z is up, relative to the torso.
        double hand_x = cos(shoulder_pitch_rad) * (ARM_UPPER_LENGTH_M * cos(shoulder_roll_rad) + ARM_LOWER_LENGTH_M * cos(shoulder_roll_rad + elbow_rad));
        double hand_y = sin(shoulder_pitch_rad) * (ARM_UPPER_LENGTH_M * cos(shoulder_roll_rad) + ARM_LOWER_LENGTH_M * cos(shoulder_roll_rad + elbow_rad));
        double hand_z = -ARM_UPPER_LENGTH_M * sin(shoulder_roll_rad) - ARM_LOWER_LENGTH_M * sin(shoulder_roll_rad + elbow_rad);

        // --- Step 3: Transform Hand Position to Camera's Coordinate Frame ---
        // These offsets are the camera's position relative to our origin (the shoulder roll joint).

        // Sideways offset (X in camera frame). Assumed to be centered.
        const double CAM_X_OFFSET_M = 0.0;

        // Vertical offset (Y in camera frame).
        // From schematic: 50.5mm from shoulder pitch axis to camera level.
        const double CAM_Y_OFFSET_M = 0.0505;

        // Forward offset (Z in camera frame).
        // From your measurement: ~3cm forward offset.
        const double CAM_Z_OFFSET_M = 0.030;

        // Calculate the hand's position from the camera's perspective by translating the origin.
        // This also involves rotating the coordinate system axes to match the camera's view.
        double hand_in_cam_x = hand_y - CAM_X_OFFSET_M; // Robot's Y-axis (left/right) is camera's X-axis.
        double hand_in_cam_y = hand_z - CAM_Y_OFFSET_M; // Robot's Z-axis (up/down) is camera's Y-axis.
        double hand_in_cam_z = hand_x - CAM_Z_OFFSET_M; // Robot's X-axis (forward/back) is camera's Z-axis.

        // --- Step 4: Project 3D Camera Point to 2D Image Plane ---
        // Using intrinsic camera data from your HeadTracking class.
        const double FOCAL_LENGTH_PX = HeadTracking::GetInstance()->camera_focal_length_px_;
        const double IMG_CENTER_X = Camera::WIDTH / 2.0;
        const double IMG_CENTER_Y = Camera::HEIGHT / 2.0;

        // Cannot project if the hand is behind or at the same plane as the camera.
        if (hand_in_cam_z <= 0)
        {
            return Point2D(-1, -1);
        }

        // Perspective Projection formula
        double image_x = (hand_in_cam_x * FOCAL_LENGTH_PX / hand_in_cam_z) + IMG_CENTER_X;
        double image_y = (hand_in_cam_y * FOCAL_LENGTH_PX / hand_in_cam_z) + IMG_CENTER_Y;

        // Calculate the final position relative to the center of the camera view.
        double final_x_rel_center = image_x - IMG_CENTER_X;
        double final_y_rel_center = image_y - IMG_CENTER_Y;

        return Point2D(final_x_rel_center, final_y_rel_center);
    }

    void RightArmController::CenterHandInView(int centering_speed) // Parameter is now used
    {
        std::cout << BOLDCYAN << "INFO: Starting visual servoing to center hand with speed " << centering_speed << "..." << RESET << std::endl;

        const int MAX_ITERATIONS = 100;
        const double PIXEL_THRESHOLD = 10.0;

        for (int i = 0; i < MAX_ITERATIONS; ++i)
        {
            // 1. Get the current position of the hand in the camera view
            Point2D pixel_error = GetHandPositionInCameraView();

            if (pixel_error.X == -1)
            {
                std::cerr << BOLDRED << "WARN: Could not get hand position. Aborting servoing." << RESET << std::endl;
                return;
            }

            std::cout << "INFO: Iteration " << i << " - Hand Error (px): X=" << pixel_error.X << ", Y=" << pixel_error.Y << std::endl;

            // 2. Check if the hand is centered
            if (std::abs(pixel_error.X) < PIXEL_THRESHOLD && std::abs(pixel_error.Y) < PIXEL_THRESHOLD)
            {
                std::cout << BOLDGREEN << "SUCCESS: Hand is centered." << RESET << std::endl;
                return; // Exit the function
            }

            // 3. Calculate the required correction in degrees
            // The P-gain converts pixel error into an angular offset.
            // The signs are inverted because a positive pixel error (hand is right/down) requires a negative angle change.
            double pitch_offset_deg = -pixel_error.Y * m_Hand_P_Gain_Y; // Y pixel error controls shoulder pitch
            double roll_offset_deg = -pixel_error.X * m_Hand_P_Gain_X;  // X pixel error controls shoulder roll

            // 4. Read the current motor positions
            int shoulder_pitch_val, shoulder_roll_val;
            cm730_->ReadWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &shoulder_pitch_val, 0);
            cm730_->ReadWord(JointData::ID_R_SHOULDER_ROLL, MX28::P_PRESENT_POSITION_L, &shoulder_roll_val, 0);

            // Convert to degrees
            double current_pitch_deg = Robot::HeadTracking::Value2Deg(shoulder_pitch_val);
            double current_roll_deg = Robot::HeadTracking::Value2Deg(shoulder_roll_val);

            // 5. Calculate new target angles
            double target_pitch_deg = current_pitch_deg + pitch_offset_deg;
            double target_roll_deg = current_roll_deg + roll_offset_deg;

            // 6. Create a new pose and apply it with the specified speed
            Pose next_pose;
            next_pose.joint_positions[JointData::ID_R_SHOULDER_PITCH] = Robot::HeadTracking::Deg2Value(target_pitch_deg);
            next_pose.joint_positions[JointData::ID_R_SHOULDER_ROLL] = Robot::HeadTracking::Deg2Value(target_roll_deg);

            // Apply the correction using the speed passed into the function
            ApplyPose(next_pose, centering_speed);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cerr << BOLDRED << "WARN: Visual servoing timed out after " << MAX_ITERATIONS << " iterations." << RESET << std::endl;
    }

    void RightArmController::SetPID(int p_gain)
    {
        int ids_to_configure[] = {
            JointData::ID_R_SHOULDER_ROLL,
            JointData::ID_R_SHOULDER_PITCH,
            JointData::ID_R_ELBOW,
            JointData::ID_R_WRIST,
            JointData::ID_R_GRIPPER,

            JointData::ID_L_SHOULDER_ROLL,
            JointData::ID_L_SHOULDER_PITCH,
            JointData::ID_L_ELBOW,
        };

        std::lock_guard<std::mutex> lock(cm730_mutex); // Protect CM730 access for multiple writes

        for (int joint_id : ids_to_configure)
        {
            cm730_->WriteByte(joint_id, MX28::P_TORQUE_ENABLE, 1, 0);
            cm730_->WriteByte(joint_id, MX28::P_P_GAIN, p_gain, 0); // P-gain values from 8 ~ 128 , more P-gain means more backlash towards the goal position.
        }

        std::cout << "INFO: RightArmController PID gains set for all joints." << std::endl;
    }
}