// LegsController.h
#ifndef LEGSCONTROLLER_H_
#define LEGSCONTROLLER_H_

#include "CM730.h"
#include "JointData.h" // Assuming JointData definition is here or accessible
#include "Globals.h"   // Assuming Pose definition is here
#include <mutex>
#include <iostream>
#include <thread>          // For std::this_thread::sleep_for
#include <chrono>          // For std::chrono::milliseconds
#include "ConsoleColors.h" // For colored output

// Forward declaration for minIni
class minIni;

namespace Robot
{
    class LegsController
    {
    public:
        LegsController(CM730 *cm730);
        virtual ~LegsController();

        void ApplyPose(const Pose &pose, int speed = 100);
        void SetPID(int p_gain = JointData::P_GAIN_DEFAULT);

        // --- Standard Poses ---
        void Stand(int moving_speed = 60, int p_gain = JointData::P_GAIN_DEFAULT);
        void ReadyToPickUpItem(int moving_speed = 50, int p_gain = JointData::P_GAIN_DEFAULT);

        // --- Walking Control Methods ---
        // Initializes walking parameters from an INI file
        void InitializeWalking(minIni *ini, const std::string &section = "Walking Config");

        // Starts walking with specified amplitudes.
        // X_AMPLITUDE: Forward(+) / Backward(-) movement.
        // Y_AMPLITUDE: Sideways Left(+) / Right(-) movement.
        // A_AMPLITUDE: Turn Left(+) / Right(-) rotation (degrees).
        void StartWalking(double x_amplitude = 0.0, double y_amplitude = 0.0, double a_amplitude = 0.0);
        void WalkForward(double x_amplitude = 20.0);   // Default to 20mm step
        void WalkBackward(double x_amplitude = -20.0); // Default to -20mm step
        void TurnLeft(double a_amplitude = 10.0);      // Default to 10 degrees turn
        void TurnRight(double a_amplitude = -10.0);    // Default to -10 degrees turn
        void StrafeLeft(double y_amplitude = 10.0);    // Default to 10mm y_amplitude
        void StrafeRight(double y_amplitude = -10.0);  // Default to -10mm y_amplitude

        // Stops walking gracefully.
        void StopWalk();

        // Checks if the walking module is currently active.
        bool IsWalking();

    private:
        CM730 *cm730_;
        std::mutex cm730_mutex;

        const Pose POSE_LEGS_DEFAULT_STAND = {
            std::map<int, int>{
                {JointData::ID_R_HIP_YAW, 2113},
                {JointData::ID_L_HIP_YAW, 1934},
                {JointData::ID_R_HIP_ROLL, 2056},
                {JointData::ID_L_HIP_ROLL, 2032},
                {JointData::ID_R_HIP_PITCH, 1753},
                {JointData::ID_L_HIP_PITCH, 2311},
                {JointData::ID_R_KNEE, 2266},
                {JointData::ID_L_KNEE, 1771},
                {JointData::ID_R_ANKLE_PITCH, 2143},
                {JointData::ID_L_ANKLE_PITCH, 1890},
                {JointData::ID_R_ANKLE_ROLL, 2105},
                {JointData::ID_L_ANKLE_ROLL, 2014}}};

        const Pose POSE_READY_TO_PICKUP_STAND = {
            std::map<int, int>{
                {JointData::ID_R_HIP_YAW, 2101},
                {JointData::ID_L_HIP_YAW, 1914},
                {JointData::ID_R_HIP_ROLL, 2012},
                {JointData::ID_L_HIP_ROLL, 2045},
                {JointData::ID_R_HIP_PITCH, 1398},
                {JointData::ID_L_HIP_PITCH, 2665},
                {JointData::ID_R_KNEE, 2628},
                {JointData::ID_L_KNEE, 1421},
                {JointData::ID_R_ANKLE_PITCH, 2222},
                {JointData::ID_L_ANKLE_PITCH, 1830},
                {JointData::ID_R_ANKLE_ROLL, 2069},
                {JointData::ID_L_ANKLE_ROLL, 2003}}};
    };

} // namespace Robot

#endif /* LEGSCONTROLLER_H_ */