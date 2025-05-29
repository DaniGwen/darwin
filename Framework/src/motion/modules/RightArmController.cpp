#include "RightArmController.h"
#include "ConsoleColors.h"

namespace Robot
{

    RightArmController::RightArmController(CM730 *cm730)
        : cm730_(cm730)
    {
        if (!cm730_)
        {
            std::cerr << "ERROR: RightArmController initialized with a NULL CM730 pointer. Motor control will not be possible." << std::endl;
        }
    }

    void RightArmController::ApplyPose(const Pose &pose)
    {
        std::lock_guard<std::mutex> lock(cm730_mutex);

        if (!cm730_)
        {
            std::cerr << "ERROR: CM730 not initialized, cannot apply pose." << std::endl;
            return;
        }

        std::cout << BLUE << "INFO: Applying pose to right arm..." << RESET << std::endl;
        for (const auto &joint_pair : pose.joint_positions)
        {
            int joint_id = joint_pair.first;
            int goal_value = joint_pair.second;

            cm730_->WriteWord(joint_id, MX28::P_GOAL_POSITION_L, goal_value, 0);
            std::cout << BLUE << "DEBUG: Set Joint ID " << joint_id << " to value " << goal_value << RESET << std::endl;
        }
    }

    void RightArmController::HandReach(int p_gain)
    {
        SetPID(p_gain);

        std::cout << "INFO: Moving right arm to POSE_RISE_HAND ..." << std::endl;
        ApplyPose(POSE_REACH_HAND);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    void RightArmController::CloseGripper(int p_gain)
    {
        SetPID(p_gain);

        std::cout << "INFO: Moving right arm to POSE_CLOSE_GRIPPER ..." << std::endl;
        ApplyPose(POSE_CLOSE_GRIPPER);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }

    void RightArmController::OpenGripper(int p_gain)
    {
        SetPID(p_gain);

        std::cout << "INFO: Moving right arm to POSE_OPEN_GRIPPER ..." << std::endl;
        ApplyPose(POSE_OPEN_GRIPPER);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }

     void RightArmController::RotateWrist90Deg(int p_gain)
    {
        SetPID(p_gain);

        std::cout << "INFO: Moving right arm to POSE_ROTATE_WRIST_90DEG ..." << std::endl;
        ApplyPose(POSE_ROTATE_WRIST_90DEG);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }

    void RightArmController::Default()
    {
        SetPID(3);

        std::cout << "INFO: Resetting right arm to default pose..." << std::endl;
        ApplyPose(DEFAULT);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
        JointData joint_data;
        for (int joint_id : ids_to_configure)
        {
            cm730_->WriteByte(joint_id, MX28::P_TORQUE_ENABLE, 1, 0);
            cm730_->WriteByte(joint_id, MX28::P_P_GAIN, p_gain, 0); // P-gain values from 8 ~ 128 , more P-gain means more backlash towards the goal position.
            cm730_->WriteWord(joint_id, MX28::P_MOVING_SPEED_L, 300, 0); // Value 0 means max speed. 1~1023 for controlled speed.
        }

        std::cout << "INFO: RightArmController PID gains set for all joints." << std::endl;
    }
}