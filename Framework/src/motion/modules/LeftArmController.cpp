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
            if (joint_id == 23 || joint_id == 24)
            {
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
            JointData::ID_L_ELBOW};

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
        if (!cm730_)
            return;

        // Reversed for the left side mirror orientation
        const int OPEN_POS = 680;

        int error = 0;

        std::lock_guard<std::mutex> lock(cm730_mutex);

        cm730_->WriteByte(JointData::ID_L_GRIPPER, MX28::P_TORQUE_ENABLE, 1, &error);
        cm730_->WriteWord(JointData::ID_L_GRIPPER, MX28::P_MOVING_SPEED_L, moving_speed, &error);

        int result = cm730_->WriteWord(JointData::ID_L_GRIPPER, MX28::P_GOAL_POSITION_L, OPEN_POS, &error);

        if (result != CM730::SUCCESS || error != 0)
        {
            std::cerr << BOLDRED << "ERROR: Failed to open Left Gripper (ID " << JointData::ID_L_GRIPPER << "). Result: "
                      << result << ", Error: " << error << RESET << std::endl;
        }
        else
        {
            std::cout << BOLDGREEN << "SUCCESS: Left Gripper (ID " << JointData::ID_L_GRIPPER << ") OPENED to position "
                      << OPEN_POS << RESET << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    void LeftArmController::CloseGripper(int moving_speed, int p_gain)
    {
        if (!cm730_)
            return;

        // Reversed for the left side mirror orientation
        const int CLOSE_POS = 410;

        int error = 0;

        std::lock_guard<std::mutex> lock(cm730_mutex);

        cm730_->WriteByte(JointData::ID_L_GRIPPER, MX28::P_TORQUE_ENABLE, 1, &error);
        cm730_->WriteWord(JointData::ID_L_GRIPPER, MX28::P_MOVING_SPEED_L, moving_speed, &error);

        int result = cm730_->WriteWord(JointData::ID_L_GRIPPER, MX28::P_GOAL_POSITION_L, CLOSE_POS, &error);

        if (result != CM730::SUCCESS || error != 0)
        {
            std::cerr << BOLDRED << "ERROR: Failed to close Left Gripper (ID " << JointData::ID_L_GRIPPER << "). Result: "
                      << result << ", Error: " << error << RESET << std::endl;
        }
        else
        {
            std::cout << BOLDGREEN << "SUCCESS: Left Gripper (ID " << JointData::ID_L_GRIPPER << ") CLOSED to position "
                      << CLOSE_POS << RESET << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    ArmTicks LeftArmController::CalculateIK(double x, double y, double z)
    {
        ArmTicks ticks = {2048, 2048, 2048, false};

        const double L1 = 69.0;
        const double L2 = 60.0;

        double d_squared = (x * x) + (y * y) + (z * z);
        double d = std::sqrt(d_squared);

        if (d >= (L1 + L2))
        {
            ticks.out_of_reach = true;
            return ticks;
        }

        double q_roll = std::atan2(y, x);
        double cos_elbow = (d_squared - (L1 * L1) - (L2 * L2)) / (2.0 * L1 * L2);
        double q_elbow = std::acos(cos_elbow);

        double angle_to_target = std::atan2(z, std::sqrt((x * x) + (y * y)));
        double cos_shoulder_interior = (d_squared + (L1 * L1) - (L2 * L2)) / (2.0 * L1 * d);
        double q_shoulder = angle_to_target + std::acos(cos_shoulder_interior);

        const double TICKS_PER_RADIAN = 2048.0 / M_PI;

        // --- MIRRORED TICK MATH ---
        // Left side servos are mounted in reverse compared to the right side
        ticks.shoulder_pitch = 2048 + static_cast<int>(q_shoulder * TICKS_PER_RADIAN);
        ticks.shoulder_roll = 2048 - static_cast<int>(q_roll * TICKS_PER_RADIAN);
        ticks.elbow_pitch = 2048 + static_cast<int>(q_elbow * TICKS_PER_RADIAN);

        // --- HARDWARE SAFETY LIMITS ---
        // Pitch: Limit to roughly +/- 90 degrees from straight down
        ticks.shoulder_pitch = std::max(1024, std::min(3072, ticks.shoulder_pitch));

        // Roll: Mirrored constraint for the left side (4096 - 2500 = 1596, 4096 - 1700 = 2396)
        ticks.shoulder_roll = std::max(1596, std::min(2396, ticks.shoulder_roll));

        // Elbow: Prevents hyperextension (bending backwards)
        ticks.elbow_pitch = std::max(1024, std::min(3072, ticks.elbow_pitch));

        return ticks;
    }

    void LeftArmController::SmoothMoveToIK(double x, double y, double z, int duration_ms)
    {
        ArmTicks target = CalculateIK(x, y, z);
        if (target.out_of_reach)
        {
            std::cout << BOLDRED << "WARN: IK Target (" << x << "," << y << "," << z << ") is out of reach!" << RESET << std::endl;
            return;
        }

        int start_pitch = 2048, start_roll = 2048, start_elbow = 2048;
        cm730_->ReadWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &start_pitch, 0);
        cm730_->ReadWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_PRESENT_POSITION_L, &start_roll, 0);
        cm730_->ReadWord(JointData::ID_L_ELBOW, MX28::P_PRESENT_POSITION_L, &start_elbow, 0);

        // --- DYNAMIC VELOCITY CAPPING ---
        int max_travel = std::max({std::abs(target.shoulder_pitch - start_pitch),
                                   std::abs(target.shoulder_roll - start_roll),
                                   std::abs(target.elbow_pitch - start_elbow)});

        const int MAX_TICKS_PER_SECOND = 1500;
        int minimum_safe_duration = (max_travel * 1000) / MAX_TICKS_PER_SECOND;

        if (duration_ms < minimum_safe_duration)
        {
            std::cout << BOLDYELLOW << "WARN: Requested IK speed too high. Overriding duration to "
                      << minimum_safe_duration << "ms to protect servos." << RESET << std::endl;
            duration_ms = minimum_safe_duration;
        }

        MotionManager::GetInstance()->SetJointEnableState(JointData::ID_L_SHOULDER_PITCH, false);
        MotionManager::GetInstance()->SetJointEnableState(JointData::ID_L_SHOULDER_ROLL, false);
        MotionManager::GetInstance()->SetJointEnableState(JointData::ID_L_ELBOW, false);

        const int MIN_SLEEP_MS = 15;
        int steps = duration_ms / MIN_SLEEP_MS;
        if (steps < 1)
            steps = 1;

        std::cout << BOLDCYAN << "INFO: Interpolating Left Arm to IK Target over " << duration_ms << "ms (" << steps << " steps)..." << RESET << std::endl;

        for (int i = 1; i <= steps; ++i)
        {
            double progress = static_cast<double>(i) / steps;

            // Cosine interpolation for soft acceleration and deceleration
            double ease = (1.0 - std::cos(progress * M_PI)) / 2.0;

            int curr_pitch = start_pitch + ((target.shoulder_pitch - start_pitch) * ease);
            int curr_roll = start_roll + ((target.shoulder_roll - start_roll) * ease);
            int curr_elbow = start_elbow + ((target.elbow_pitch - start_elbow) * ease);

            cm730_->WriteWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L, curr_pitch, 0);
            cm730_->WriteWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L, curr_roll, 0);
            cm730_->WriteWord(JointData::ID_L_ELBOW, MX28::P_GOAL_POSITION_L, curr_elbow, 0);

            std::this_thread::sleep_for(std::chrono::milliseconds(MIN_SLEEP_MS));
        }
    }
}