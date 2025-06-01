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

        void ApplyPose(const Pose &pose, int speed = 100, int p_gain = JointData::P_GAIN_DEFAULT);
        void SetPID(int p_gain = JointData::P_GAIN_DEFAULT);

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
    };

} // namespace Robot

#endif /* LEGSCONTROLLER_H_ */