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

        void Stand(int p_gain = 25);
        void ReadyToPickUpItem(int p_gain = 25);

    private:
        CM730 *cm730_; // Pointer to the CM730 board interface

        const Pose POSE_LEGS_DEFAULT_STAND = {
            std::map<int, int>{
                {JointData::ID_R_HIP_YAW, 2165},
                {JointData::ID_L_HIP_YAW, 1964},
                {JointData::ID_R_HIP_ROLL, 2080},
                {JointData::ID_L_HIP_ROLL, 1984},
                {JointData::ID_R_HIP_PITCH, 1863},
                {JointData::ID_L_HIP_PITCH, 2220},
                {JointData::ID_R_KNEE, 2217},
                {JointData::ID_L_KNEE, 1841},
                {JointData::ID_R_ANKLE_PITCH, 2119},
                {JointData::ID_L_ANKLE_PITCH, 1960},
                {JointData::ID_R_ANKLE_ROLL, 2120},
                {JointData::ID_L_ANKLE_ROLL, 1978}}};

        const Pose POSE_READY_TO_PICKUP_STAND = {
            std::map<int, int>{
                {JointData::ID_R_HIP_YAW, 2165},
                {JointData::ID_L_HIP_YAW, 1964},
                {JointData::ID_R_HIP_ROLL, 2080},
                {JointData::ID_L_HIP_ROLL, 1984},
                {JointData::ID_R_HIP_PITCH, 1239},
                {JointData::ID_L_HIP_PITCH, 2840},
                {JointData::ID_R_KNEE, 2569},
                {JointData::ID_L_KNEE, 1494},
                {JointData::ID_R_ANKLE_PITCH, 2195},
                {JointData::ID_L_ANKLE_PITCH, 1865},
                {JointData::ID_R_ANKLE_ROLL, 2120},
                {JointData::ID_L_ANKLE_ROLL, 1978}}};

        void ApplyPose(const Pose &pose);
        void SetPID(int p_gain = 25);
    };
}

#endif // LEGS_CONTROLLER_H_