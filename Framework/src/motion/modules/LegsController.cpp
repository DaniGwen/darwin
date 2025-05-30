// LegsController.cpp
#include "LegsController.h"
#include "MX28.h"
#include "Walking.h" // Include the Walking header
#include "minIni.h"  // For minIni
#include <vector>

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

        // It's good practice to initialize the Walking module once if LegsController manages its lifecycle parameters
        // However, Walking::Initialize is often called by MotionManager or a global init.
        // For now, we'll add a specific InitializeWalking method.
    }

    LegsController::~LegsController()
    {
        std::cout << BOLDGREEN << "INFO: LegsController destroyed." << RESET << std::endl;
    }

    void LegsController::ApplyPose(const Pose &pose, int speed)
    {
        if (!cm730_)
        {
            std::cerr << BOLDRED << "ERROR: CM730 not initialized in LegsController, cannot apply pose." << RESET << std::endl;
            return;
        }

        if (IsWalking())
        {
            std::cout << BOLDYELLOW << "WARNING: Walking was active. Stopping walk before applying static pose." << RESET << std::endl;
            StopWalk();
            // Give some time for the walking module to actually stop and for MotionManager to process it
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(Walking::GetInstance()->m_PeriodTime * 1.5)));
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

    void LegsController::SetPID(int p_gain)
    {
        if (!cm730_)
            return;

        int leg_joint_ids[] = {
            JointData::ID_R_HIP_YAW, JointData::ID_R_HIP_ROLL, JointData::ID_R_HIP_PITCH,
            JointData::ID_R_KNEE, JointData::ID_R_ANKLE_PITCH, JointData::ID_R_ANKLE_ROLL,
            JointData::ID_L_HIP_YAW, JointData::ID_L_HIP_ROLL, JointData::ID_L_HIP_PITCH,
            JointData::ID_L_KNEE, JointData::ID_L_ANKLE_PITCH, JointData::ID_L_ANKLE_ROLL};

        std::lock_guard<std::mutex> lock(cm730_mutex);
        for (int joint_id : leg_joint_ids)
        {
            cm730_->WriteByte(joint_id, MX28::P_TORQUE_ENABLE, 1, 0);
            cm730_->WriteByte(joint_id, MX28::P_P_GAIN, p_gain, 0);
        }
        // std::cout << BOLDGREEN << "INFO: LegsController PID gains set (P=" << p_gain << ") and speed (" << moving_speed << ") for all leg joints." << RESET << std::endl;
    }

    void LegsController::Stand(int moving_speed, int p_gain)
    {
        std::cout << BOLDGREEN << "INFO: LegsController moving to default standing pose..." << RESET << std::endl;
        StopWalk();                                                  // Ensure walking is stopped
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Small delay

        SetPID(p_gain);
        ApplyPose(POSE_LEGS_DEFAULT_STAND, moving_speed);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    void LegsController::ReadyToPickUpItem(int moving_speed, int p_gain)
    {
        std::cout << BOLDGREEN << "INFO: LegsController moving to ReadyToPickUpItem pose..." << RESET << std::endl;
        StopWalk(); // Ensure walking is stopped
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        SetPID(p_gain);
        ApplyPose(POSE_READY_TO_PICKUP_STAND, moving_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    // --- Walking Control Method Implementations ---
    void LegsController::InitializeWalking(minIni *ini, const std::string &section)
    {
        std::cout << BOLDGREEN << "INFO: Initializing Walking module through LegsController..." << RESET << std::endl;
        Walking::GetInstance()->Initialize();
        if (ini)
        {
            Walking::GetInstance()->LoadINISettings(ini, section);
        }
        else
        {
            std::cout << BOLDYELLOW << "WARNING: minIni object not provided to InitializeWalking. Using default walking parameters." << RESET << std::endl;
        }
        // Walking parameters are set, but walking doesn't start until StartWalking() or specific walk methods are called.
    }

    void LegsController::StartWalking(double x_amplitude, double y_amplitude, double a_amplitude)
    {
        std::cout << BOLDGREEN << "INFO: LegsController commanding StartWalk. X=" << x_amplitude << ", Y=" << y_amplitude << ", A=" << a_amplitude << RESET << std::endl;
        Walking *walking = Walking::GetInstance();
        walking->PERIOD_TIME = 700; // Set the period time for walking
        walking->X_MOVE_AMPLITUDE = x_amplitude;
        walking->Y_MOVE_AMPLITUDE = y_amplitude;
        walking->A_MOVE_AMPLITUDE = a_amplitude;
        walking->Start();
    }

    void LegsController::WalkForward(double x_amplitude)
    {
        StartWalking(x_amplitude, 0.0, 0.0);
    }

    void LegsController::WalkBackward(double x_amplitude)
    {
        StartWalking(x_amplitude, 0.0, 0.0); // Negative x_amplitude for backward
    }

    void LegsController::TurnLeft(double a_amplitude)
    {
        StartWalking(0.0, 0.0, a_amplitude); // Positive a_amplitude for left turn
    }

    void LegsController::TurnRight(double a_amplitude)
    {
        StartWalking(0.0, 0.0, a_amplitude); // Negative a_amplitude for right turn
    }

    void LegsController::StrafeLeft(double y_amplitude)
    {
        StartWalking(0.0, y_amplitude, 0.0); // Positive y_amplitude for Y-axis (strafe left)
    }

    void LegsController::StrafeRight(double y_amplitude)
    {
        StartWalking(0.0, y_amplitude, 0.0); // Negative y_amplitude for Y-axis (strafe right)
    }

    void LegsController::StopWalk()
    {
        std::cout << BOLDGREEN << "INFO: LegsController commanding StopWalk." << RESET << std::endl;
        Walking *walking = Walking::GetInstance();
        // Setting amplitudes to 0 directly is more immediate if Stop() has a delay in processing.
        // walking->X_MOVE_AMPLITUDE = 0;
        // walking->Y_MOVE_AMPLITUDE = 0;
        // walking->A_MOVE_AMPLITUDE = 0;
        // The Walking::Stop() method sets m_Ctrl_Running = false,
        // which then causes the Process() method to set amplitudes to 0 when m_Time == 0.
        walking->Stop(); //

        // It might take one or two walking cycles for m_Real_Running to become false.
        // If PID gains need to be reset immediately after stopping, consider that delay.
        // For example, after calling StopWalk(), you might want to revert to default PIDs for standing.
        // SetPID(); // Revert to default PIDs after stopping walk
    }

    bool LegsController::IsWalking()
    {
        return Walking::GetInstance()->IsRunning();
    }

} // namespace Robot