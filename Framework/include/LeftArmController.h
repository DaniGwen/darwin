#ifndef LEFT_ARM_CONTROLLER_H_
#define LEFT_ARM_CONTROLLER_H_

#include "CM730.h"     // For CM730 class to control motors
#include "JointData.h" // For JointData::ID_HEAD_PAN, ID_HEAD_TILT, MX28 constants (and other joint IDs)
#include <iostream>    // For std::cout, std::cerr
#include <vector>      // For std::vector
#include <map>         // For std::map to store joint poses
#include <thread>      // For std::this_thread::sleep_for
#include <chrono>      // For std::chrono::milliseconds

namespace Robot
{

    // Structure to define a single arm pose
    struct ArmPose
    {
        std::map<int, int> joint_positions; // Map of Joint ID to Goal Position Value
    };

    class LeftArmController
    {
    public:
        LeftArmController(CM730 *cm730);

        // repetitions: The number of times the entire sequence (Pose 1 -> Pose 2) should repeat.
        // delay_ms: The delay in milliseconds between each pose movement to allow motors to reach their positions.
        void Wave(int repetitions = 3, int delay_ms = 1000);

    private:
        CM730 *cm730_; // Pointer to the CM730 instance for direct motor control

        // Define the two poses for the left arm
        const ArmPose POSE_1 = {
            std::map<int, int>{
                {JointData::ID_L_SHOULDER_ROLL, 1251},  // Example: Assuming ID_L_SHOULDER_ROLL is Joint ID 2
                {JointData::ID_L_SHOULDER_PITCH, 1670}, // Example: Assuming ID_L_ELBOW is Joint ID 4
                {JointData::ID_L_ELBOW, 1869}           // Example: Assuming ID_L_WRIST_YAW is Joint ID 6
            }};

        const ArmPose POSE_2 = {
            std::map<int, int>{
                {JointData::ID_L_SHOULDER_ROLL, 1251},  // Example: Assuming ID_L_SHOULDER_ROLL is Joint ID 2
                {JointData::ID_L_SHOULDER_PITCH, 1670}, // Example: Assuming ID_L_ELBOW is Joint ID 4
                {JointData::ID_L_ELBOW, 2433}           // Example: Assuming ID_L_WRIST_YAW is Joint ID 6
            }};

        // Helper method to apply a single arm pose
        void ApplyPose(const ArmPose &pose);
        void InitializeLeftArm();
    };

}

#endif // LEFT_ARM_CONTROLLER_H_