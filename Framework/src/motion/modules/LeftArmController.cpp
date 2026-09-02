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

     void LeftArmController::ApplyPose(const Pose &pose, int speed)
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

            // --- VIRTUALIZATION FIX FOR AX-18 ---
            // If the joint is the left hand (23 or 24), scale the 4095 value down to 1023
            if (joint_id == 23 || joint_id == 24) {
                goal_value /= 4; 
            }

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

    void LeftArmController::ToDefaultPose()
    {
        SetPID();

        std::cout << "INFO: Resetting left arm to default pose..." << std::endl;
        ApplyPose(DEFAULT);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    void LeftArmController::SetPID(int p_gain)
    {
        if (!cm730_)
        {
            std::cerr << "ERROR: CM730 not initialized, cannot initialize left arm." << std::endl;
            return;
        }

        // Configure standard arm MX-28 joints
        int arm_joints[] = {
            JointData::ID_L_SHOULDER_ROLL,
            JointData::ID_L_SHOULDER_PITCH,
            JointData::ID_L_ELBOW
        };

        std::lock_guard<std::mutex> lock(cm730_mutex);

        for (int joint_id : arm_joints)
        {
            int error = 0;
            cm730_->WriteByte(joint_id, MX28::P_TORQUE_ENABLE, 1, &error);
            cm730_->WriteByte(joint_id, MX28::P_P_GAIN, p_gain, &error);

            if (error != CM730::SUCCESS)
            {
                std::cerr << "ERROR: Failed to configure Joint ID " << joint_id << std::endl;
                return;
            }
        }

        // Explicitly enable torque for the AX-18 left gripper
        int error = 0;
        cm730_->WriteByte(JointData::ID_L_GRIPPER, MX28::P_TORQUE_ENABLE, 1, &error);

        std::cout << "INFO: LeftArmController initialized." << std::endl;
    }

   void LeftArmController::OpenGripper(int moving_speed, int p_gain)
    {
        if (!cm730_) return;

        const int LEFT_GRIPPER_ID = JointData::ID_L_GRIPPER;
        // AX-18 range is 0-1023 (380 is ~1516 / 4)
        const int OPEN_POS = 380; 

        int error = 0;

        std::lock_guard<std::mutex> lock(cm730_mutex);

        // 1. Enable Torque (Address 24)
        cm730_->WriteByte(LEFT_GRIPPER_ID, MX28::P_TORQUE_ENABLE, 1, &error);

        // 2. Set Moving Speed (Address 32)
        cm730_->WriteWord(LEFT_GRIPPER_ID, MX28::P_MOVING_SPEED_L, moving_speed, &error);

        // 3. Command Goal Position (Address 30)
        int result = cm730_->WriteWord(LEFT_GRIPPER_ID, MX28::P_GOAL_POSITION_L, OPEN_POS, &error);

        if (result != CM730::SUCCESS || error != 0)
        {
            std::cerr << BOLDRED << "ERROR: Failed to open Left Gripper (ID 24). Result: " 
                      << result << ", Error byte: " << error << RESET << std::endl;
        }
        else
        {
            std::cout << BOLDGREEN << "SUCCESS: Left Gripper (ID 24) moved to position " 
                      << OPEN_POS << RESET << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    void LeftArmController::CloseGripper(int moving_speed, int p_gain)
    {
        if (!cm730_) return;

        const int LEFT_GRIPPER_ID = JointData::ID_L_GRIPPER;
        // AX-18 range is 0-1023 (527 is ~2109 / 4)
        const int CLOSE_POS = 527; 

        int error = 0;

        std::lock_guard<std::mutex> lock(cm730_mutex);

        // 1. Enable Torque (Address 24)
        cm730_->WriteByte(LEFT_GRIPPER_ID, MX28::P_TORQUE_ENABLE, 1, &error);

        // 2. Set Moving Speed (Address 32)
        cm730_->WriteWord(LEFT_GRIPPER_ID, MX28::P_MOVING_SPEED_L, moving_speed, &error);

        // 3. Command Goal Position (Address 30)
        int result = cm730_->WriteWord(LEFT_GRIPPER_ID, MX28::P_GOAL_POSITION_L, CLOSE_POS, &error);

        if (result != CM730::SUCCESS || error != 0)
        {
            std::cerr << BOLDRED << "ERROR: Failed to close Left Gripper (ID 24). Result: " 
                      << result << ", Error byte: " << error << RESET << std::endl;
        }
        else
        {
            std::cout << BOLDGREEN << "SUCCESS: Left Gripper (ID 24) moved to position " 
                      << CLOSE_POS << RESET << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}