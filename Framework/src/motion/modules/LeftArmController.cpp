#include "LeftArmController.h"

namespace Robot
{

    LeftArmController::LeftArmController(CM730 *cm730)
        : cm730_(cm730)
    {
        if (!cm730_)
        {
            std::cerr << "ERROR: LeftArmController initialized with a NULL CM730 pointer. Motor control will not be possible." << std::endl;
        }
    }

    void LeftArmController::ApplyPose(const ArmPose &pose)
    {
        if (!cm730_)
        {
            std::cerr << "ERROR: CM730 not initialized, cannot apply pose." << std::endl;
            return;
        }

        std::cout << "INFO: Applying pose..." << std::endl;
        for (const auto &joint_pair : pose.joint_positions)
        {
            int joint_id = joint_pair.first;
            int goal_value = joint_pair.second;

            cm730_->WriteWord(joint_id, MX28::P_GOAL_POSITION_L, goal_value, 0);
            std::cout << "DEBUG: Set Joint ID " << joint_id << " to value " << goal_value << std::endl;
        }
    }

    void LeftArmController::Wave(int repetitions, int delay_ms)
    {
        InitializeLeftArm();

        if (!cm730_)
        {
            std::cerr << "ERROR: CM730 not initialized, cannot run arm sequence." << std::endl;
            return;
        }

        std::cout << "INFO: Starting left arm movement sequence for " << repetitions << " repetitions." << std::endl;

        for (int i = 0; i < repetitions; ++i)
        {
            std::cout << "INFO: Repetition " << (i + 1) << "/" << repetitions << std::endl;

            // Move to Pose 1
            std::cout << "INFO: Moving to Pose 1..." << std::endl;
            ApplyPose(POSE_1);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms)); // Wait for motors to move

            // Move to Pose 2
            std::cout << "INFO: Moving to Pose 2..." << std::endl;
            ApplyPose(POSE_2);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms)); // Wait for motors to move
        }

        std::cout << "INFO: Left arm movement sequence finished." << std::endl;
    }

    void LeftArmController::InitializeLeftArm(int p_gain, int d_gain)
    {
        if (!cm730_)
        {
            std::cerr << "ERROR: CM730 not initialized, cannot initialize left arm." << std::endl;
            return;
        }
        cm730_->WriteByte(JointData::ID_L_SHOULDER_ROLL, MX28::P_TORQUE_ENABLE, 1, 0);
        cm730_->WriteByte(JointData::ID_L_SHOULDER_PITCH, MX28::P_TORQUE_ENABLE, 1, 0);
        cm730_->WriteByte(JointData::ID_L_ELBOW, MX28::P_TORQUE_ENABLE, 1, 0);

        cm730_->WriteByte(JointData::ID_L_SHOULDER_ROLL, MX28::P_P_GAIN, p_gain, 0);
        cm730_->WriteByte(JointData::ID_L_SHOULDER_PITCH, MX28::P_P_GAIN, p_gain, 0);
        cm730_->WriteByte(JointData::ID_L_ELBOW, MX28::P_P_GAIN, p_gain, 0);

        cm730_->WriteByte(JointData::ID_L_SHOULDER_ROLL, MX28::P_D_GAIN, d_gain, 0);
        cm730_->WriteByte(JointData::ID_L_SHOULDER_PITCH, MX28::P_D_GAIN, d_gain, 0);
        cm730_->WriteByte(JointData::ID_L_ELBOW, MX28::P_D_GAIN, d_gain, 0);

        std::cout << "INFO: LeftArmController initialized." << std::endl;
    }
}