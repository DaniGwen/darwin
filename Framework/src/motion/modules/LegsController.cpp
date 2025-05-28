#include "LegsController.h"
#include "MX28.h" // For MX28::P_GOAL_POSITION_L, MX28::P_P_GAIN etc.

namespace Robot
{

    LegsController::LegsController(CM730 *cm730)
        : cm730_(cm730)
    {
        if (!cm730_)
        {
            std::cerr << BOLDRED << "ERROR: LegsController initialized with a NULL CM730 pointer. Motor control will not be possible." << RESET << std::endl;
        }
        std::cout << BOLDGREEN << "INFO: LegsController initialized." << RESET << std::endl;

        SetPID(); // Use default PID gains initially
    }

    LegsController::~LegsController()
    {
        std::cout << BOLDGREEN << "INFO: LegsController destroyed." << RESET << std::endl;
    }

    void LegsController::ApplyPose(const Pose &pose)
    {
        if (!cm730_)
        {
            std::cerr << BOLDRED << "ERROR: CM730 not initialized in LegsController, cannot apply pose." << RESET << std::endl;
            return;
        }

        std::lock_guard<std::mutex> lock(cm730_mutex);

        std::cout << BOLDCYAN << "INFO: LegsController applying pose..." << RESET << std::endl;
        for (const auto &joint_pair : pose.joint_positions)
        {
            int joint_id = joint_pair.first;
            int goal_value = joint_pair.second;

            // Basic validation for joint ID (optional, but good practice)
            if (joint_id < JointData::ID_R_HIP_YAW || joint_id > JointData::ID_L_ANKLE_ROLL)
            {
                // This check assumes leg joint IDs are in a contiguous block or known range
                // std::cerr << BOLDYELLOW << "WARNING: LegsController applying pose to unexpected joint ID: " << joint_id << RESET << std::endl;
            }

            cm730_->WriteWord(joint_id, MX28::P_GOAL_POSITION_L, goal_value, 0);
            // std::cout << BOLDCYAN << "DEBUG: LegsController - Set Joint ID " << joint_id << " to value " << goal_value << RESET << std::endl;
        }
    }

    void LegsController::Stand(int p_gain)
    {
        std::cout << BOLDGREEN << "INFO: LegsController moving to default standing pose..." << RESET << std::endl;
        SetPID(p_gain); // Set P-gain for this movement
        ApplyPose(POSE_LEGS_DEFAULT_STAND);

        std::this_thread::sleep_for(std::chrono::milliseconds(3000)); // e.g., 1.5 seconds
    }

    void LegsController::ReadyToPickUpItem(int p_gain)
    {
        std::cout << BOLDGREEN << "INFO: LegsController moving to ReadyToPickUpItem pose..." << RESET << std::endl;
        SetPID(p_gain);
        ApplyPose(POSE_READY_TO_PICKUP_STAND);

        std::this_thread::sleep_for(std::chrono::milliseconds(3500)); // e.g., 1.5 seconds
    }

    void LegsController::SetPID(int p_gain)
    {
        // Define all leg joint IDs that need PID configuration
        int leg_joint_ids[] = {
            JointData::ID_R_HIP_YAW,
            JointData::ID_R_HIP_ROLL,
            JointData::ID_R_HIP_PITCH,
            JointData::ID_R_KNEE,
            JointData::ID_R_ANKLE_PITCH,
            JointData::ID_R_ANKLE_ROLL,
            JointData::ID_L_HIP_YAW,
            JointData::ID_L_HIP_ROLL,
            JointData::ID_L_HIP_PITCH,
            JointData::ID_L_KNEE,
            JointData::ID_L_ANKLE_PITCH,
            JointData::ID_L_ANKLE_ROLL};

        std::lock_guard<std::mutex> lock(cm730_mutex); // Protect CM730 access

        for (int joint_id : leg_joint_ids)
        {
            cm730_->WriteByte(joint_id, MX28::P_TORQUE_ENABLE, 1, 0); // Ensure torque is enabled
            cm730_->WriteByte(joint_id, MX28::P_P_GAIN, p_gain, 0);
        }

        // std::cout << BOLDGREEN << "INFO: LegsController PID gains set (P=" << p_gain << ", I=" << i_gain << ", D=" << d_gain << ") for all leg joints." << RESET << std::endl;
    }

} // namespace Robot