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

    void LegsController::ApplyPose(const Pose &pose)
    {
        if (!cm730_)
        {
            std::cerr << BOLDRED << "ERROR: CM730 not initialized in LegsController, cannot apply pose." << RESET << std::endl;
            return;
        }

        // Ensure walking is stopped before applying a static pose to avoid conflicts
        if (IsWalking())
        {
            std::cout << BOLDYELLOW << "WARNING: Walking was active. Stopping walk before applying static pose." << RESET << std::endl;
            StopWalk();
            // Give some time for the walking module to actually stop and for MotionManager to process it
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(Walking::GetInstance()->m_PeriodTime * 1.5)));
        }

        const int DEFAULT_P_GAIN = 32;
        const int DEFAULT_I_GAIN = 0;
        const int DEFAULT_D_GAIN = 0;
        const int RESERVED_BYTE = 0;

        // The data length per motor is 6 bytes (D, I, P, Reserved, Pos_L, Pos_H)
        const int DATA_LENGTH_PER_MOTOR = 6;

        std::lock_guard<std::mutex> lock(cm730_mutex);

        // Step 1: Prepare the full data packet exactly like MotionManager
        std::vector<int> params;
        // Each motor needs 1 byte for ID + DATA_LENGTH_PER_MOTOR bytes for data
        params.reserve(pose.joint_positions.size() * (1 + DATA_LENGTH_PER_MOTOR));

        for (const auto &joint_pair : pose.joint_positions)
        {
            int joint_id = joint_pair.first;
            int goal_value = joint_pair.second;

            params.push_back(joint_id);
            params.push_back(DEFAULT_D_GAIN);
            params.push_back(DEFAULT_I_GAIN);
            params.push_back(DEFAULT_P_GAIN);
            params.push_back(RESERVED_BYTE);
            params.push_back(CM730::GetLowByte(goal_value));
            params.push_back(CM730::GetHighByte(goal_value));
        }

        // Step 2: Send the single SyncWrite command with the correct parameters
        if (!params.empty())
        {
            std::cout << BOLDCYAN << "INFO: LegsController applying pose with CORRECT SyncWrite packet..." << RESET << std::endl;

            int num_joints = params.size() / (1 + DATA_LENGTH_PER_MOTOR);

            // The starting address is P_D_GAIN, and the length is 6 bytes
            int result = cm730_->SyncWrite(MX28::P_D_GAIN, DATA_LENGTH_PER_MOTOR, num_joints, params.data());

            if (result != cm730_->SUCCESS)
            {
                // Handle the error case
                std::cerr << BOLDRED << "ERROR: SyncWrite failed with communication status: " << result << RESET << std::endl;
            }
            else
            {
                std::cout << BOLDGREEN << "INFO: Pose applied successfully." << RESET << std::endl;
            }
        }
    }

    void LegsController::SetPID(int moving_speed, int p_gain)
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
            cm730_->WriteWord(joint_id, MX28::P_MOVING_SPEED_L, moving_speed, 0);
        }
        // std::cout << BOLDGREEN << "INFO: LegsController PID gains set (P=" << p_gain << ") and speed (" << moving_speed << ") for all leg joints." << RESET << std::endl;
    }

    void LegsController::Stand(int moving_speed, int p_gain)
    {
        std::cout << BOLDGREEN << "INFO: LegsController moving to default standing pose..." << RESET << std::endl;
        StopWalk();                                                  // Ensure walking is stopped
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Small delay

        SetPID(moving_speed, p_gain);
        ApplyPose(POSE_LEGS_DEFAULT_STAND);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    void LegsController::ReadyToPickUpItem(int moving_speed, int p_gain)
    {
        std::cout << BOLDGREEN << "INFO: LegsController moving to ReadyToPickUpItem pose..." << RESET << std::endl;
        StopWalk(); // Ensure walking is stopped
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        SetPID(moving_speed, p_gain);
        ApplyPose(POSE_READY_TO_PICKUP_STAND);
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
        return Walking::GetInstance()->IsRunning(); //
    }

} // namespace Robot