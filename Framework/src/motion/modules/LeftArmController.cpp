#include "LeftArmController.h"
#include "ConsoleColors.h"

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

    void LeftArmController::ApplyPose(const Pose &pose)
    {
        std::lock_guard<std::mutex> lock(cm730_mutex); // Protect CM730 access

        std::cout << "INFO: Applying pose..." << std::endl;

        for (const auto &joint_pair : pose.joint_positions)
        {
            int joint_id = joint_pair.first;
            int goal_value = joint_pair.second;

            cm730_->WriteWord(joint_id, MX28::P_GOAL_POSITION_L, goal_value, 0);
            std::cout << "DEBUG: Set Joint ID " << joint_id << " to value " << goal_value << std::endl;
        }
    }

    void LeftArmController::Wave(int moving_speed, int repetitions, int p_gain)
    {
        std::cout << "INFO: Starting left arm movement sequence for " << repetitions << " repetitions." << std::endl;

        SetPID(moving_speed, p_gain);

        for (int i = 0; i < repetitions; ++i)
        {
            ApplyPose(POSE_1);
            std::this_thread::sleep_for(std::chrono::milliseconds(700));
            ApplyPose(POSE_2);
            std::this_thread::sleep_for(std::chrono::milliseconds(700));
        }

        std::cout << GREEN << "INFO: Wave movement sequence finished." << RESET << std::endl;
    }

    void LeftArmController::ToDefaultPose()
    {
        SetPID();

        std::cout << "INFO: Resetting left arm to default pose..." << std::endl;
        ApplyPose(DEFAULT);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    void LeftArmController::SetPID(int moving_speed, int p_gain)
    {
        if (!cm730_)
        {
            std::cerr << "ERROR: CM730 not initialized, cannot initialize left arm." << std::endl;
            return;
        }

        int ids_to_configure[] = {
            JointData::ID_L_SHOULDER_ROLL,
            JointData::ID_L_SHOULDER_PITCH,
            JointData::ID_L_ELBOW};

        std::lock_guard<std::mutex> lock(cm730_mutex); // Protect CM730 access for multiple writes

        for (int joint_id : ids_to_configure)
        {
            int error = 0;
            cm730_->WriteByte(joint_id, MX28::P_TORQUE_ENABLE, 1, &error);
            cm730_->WriteByte(joint_id, MX28::P_P_GAIN, p_gain, &error); // P-gain values from 8 ~ 128 , more P-gain means more backlash towards the goal position.
            cm730_->WriteWord(joint_id, MX28::P_MOVING_SPEED_L, moving_speed, &error);

            if (error != CM730::SUCCESS)
            {
                std::cerr << "ERROR: Failed to enable torque for Joint ID " << joint_id << std::endl;
                return;
            }
        }

        std::cout << "INFO: LeftArmController initialized." << std::endl;
    }
}