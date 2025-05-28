#ifndef LEGS_CONTROLLER_H_
#define LEGS_CONTROLLER_H_

#include "CM730.h"         // For CM730 class to control motors
#include "JointData.h"     // For JointData constants
#include "Globals.h"       // For ArmPose (assuming it's a generic pose type) and cm730_mutex
#include "ConsoleColors.h" // For styled console output

#include <iostream> // For std::cout, std::cerr
#include <vector>   // For std::vector
#include <map>      // For std::map to store joint poses
#include <thread>   // For std::this_thread::sleep_for
#include <chrono>   // For std::chrono::milliseconds
#include <mutex>    // For std::lock_guard

namespace Robot
{
    // If Pose is specific to arms, you might want a more generic Pose struct in Globals.h
    // For now, we'll reuse Pose, assuming its structure is:
    // typedef struct { std::map<int, int> joint_positions; } Pose;
    // If not, adjust accordingly or define a LegPose.

    class LegsController
    {
    public:
        LegsController(CM730 *cm730);
        ~LegsController();

        void ToDefaultPose(int p_gain = 4);
        void ReadyToPickUpItem(int p_gain = 4);

    private:
        CM730 *cm730_; // Pointer to the CM730 board interface

        const Pose POSE_LEGS_DEFAULT_STAND = {
            std::map<int, int>{
                // Right Leg
                {JointData::ID_R_HIP_YAW, 2048},     // Neutral yaw
                {JointData::ID_R_HIP_ROLL, 2048},    // Neutral roll (adjust for balance)
                {JointData::ID_R_HIP_PITCH, 3072},   // Slight bend forward at hip (e.g., Robotis uses ~2600-3000 for stand)
                {JointData::ID_R_KNEE, 1024},        // Knee bent (e.g., Robotis uses ~1000-1500)
                {JointData::ID_R_ANKLE_PITCH, 3072}, // Ankle to keep foot flat (e.g., Robotis uses ~2600-3000)
                {JointData::ID_R_ANKLE_ROLL, 2048},  // Neutral roll (adjust for balance)

                // Left Leg
                {JointData::ID_L_HIP_YAW, 2048},     // Neutral yaw
                {JointData::ID_L_HIP_ROLL, 2048},    // Neutral roll (adjust for balance)
                {JointData::ID_L_HIP_PITCH, 1024},   // Slight bend forward at hip (mirrors right, adjust)
                {JointData::ID_L_KNEE, 3072},        // Knee bent (mirrors right, adjust)
                {JointData::ID_L_ANKLE_PITCH, 1024}, // Ankle to keep foot flat (mirrors right, adjust)
                {JointData::ID_L_ANKLE_ROLL, 2048}   // Neutral roll (adjust for balance)
            }};

        const Pose POSE_READY_TO_PICKUP_STAND = {
            std::map<int, int>{
                // Right Leg
                {JointData::ID_R_HIP_YAW, 2165},     // Neutral yaw
                {JointData::ID_R_HIP_ROLL, 2080},    // Neutral roll (adjust for balance)
                {JointData::ID_R_HIP_PITCH, 1239},   // Slight bend forward at hip (e.g., Robotis uses ~2600-3000 for stand)
                {JointData::ID_R_KNEE, 2569},        // Knee bent (e.g., Robotis uses ~1000-1500)
                {JointData::ID_R_ANKLE_PITCH, 2129}, // Ankle to keep foot flat (e.g., Robotis uses ~2600-3000)
                {JointData::ID_R_ANKLE_ROLL, 2120},  // Neutral roll (adjust for balance)

                // Left Leg
                {JointData::ID_L_HIP_YAW, 1964},     // Neutral yaw
                {JointData::ID_L_HIP_ROLL, 1984},    // Neutral roll (adjust for balance)
                {JointData::ID_L_HIP_PITCH, 2840},   // Slight bend forward at hip (mirrors right, adjust)
                {JointData::ID_L_KNEE, 1494},        // Knee bent (mirrors right, adjust)
                {JointData::ID_L_ANKLE_PITCH, 1950}, // Ankle to keep foot flat (mirrors right, adjust)
                {JointData::ID_L_ANKLE_ROLL, 1978}   // Neutral roll (adjust for balance)
            }};

        void ApplyPose(const Pose &pose);
        void SetPIDGains(int p_gain = 4, int i_gain = 0, int d_gain = 0);
    };
}

#endif // LEGS_CONTROLLER_H_