#ifndef LEFT_ARM_CONTROLLER_H_
#define LEFT_ARM_CONTROLLER_H_

#include "CM730.h"     // For CM730 class to control motors
#include "JointData.h" // For JointData::ID_HEAD_PAN, ID_HEAD_TILT, MX28 constants (and other joint IDs)
#include <iostream>    // For std::cout, std::cerr
#include <vector>      // For std::vector
#include <map>         // For std::map to store joint poses
#include <thread>      // For std::this_thread::sleep_for
#include <chrono>      // For std::chrono::milliseconds
#include "Globals.h"

namespace Robot
{
    class LeftArmController
    {
    public:
        LeftArmController(CM730 *cm730);
        void ToDefaultPose();

    private:
        CM730 *cm730_; // Pointer to the CM730 instance for direct motor control

        const Pose DEFAULT = {
            std::map<int, int>{
                {JointData::ID_L_SHOULDER_PITCH, 2423}, // Default position for left shoulder pitch
                {JointData::ID_L_SHOULDER_ROLL, 2370},  // Default position for left shoulder roll
                {JointData::ID_L_ELBOW, 2336}           // Default position for left elbow
            }};

        // Helper method to apply a single arm pose
        void ApplyPose(const Pose &pose, int speed = 100);
        void SetPID(int p_gain = 15);
    };
}

#endif