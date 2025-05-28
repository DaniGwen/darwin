#ifndef RIGHT_ARM_CONTROLLER_H_
#define RIGHT_ARM_CONTROLLER_H_

#include "CM730.h"     // For CM730 class to control motors
#include "JointData.h" // For JointData constants (including new right arm joint IDs)
#include <iostream>    // For std::cout, std::cerr
#include <vector>      // For std::vector
#include <map>         // For std::map to store joint poses
#include <thread>      // For std::this_thread::sleep_for
#include <chrono>      // For std::chrono::milliseconds
#include "Globals.h"

namespace Robot
{
    class RightArmController
    {
    public:
        RightArmController(CM730 *cm730);

        void RiseHand(int p_gain = 4);
        void GrabItem(int p_gain = 4);
        void ToDefaultPose();
        void SetGripper(int position, int p_gain = 4, int d_gain = 4);

    private:
        CM730 *cm730_;

        // Define poses for the right arm including wrist and gripper
        // Assuming JointData::ID_R_WRIST = 21 and JointData::ID_R_GRIPPER = 22 are defined in JointData.h
        // Placeholder values for wrist and gripper have been added. Adjust as needed.
        const ArmPose POSE_REACH_HAND = {
            std::map<int, int>{
                {JointData::ID_R_SHOULDER_PITCH, 2376},
                {JointData::ID_R_SHOULDER_ROLL, 1749},
                {JointData::ID_R_ELBOW, 1628},

                {JointData::ID_L_SHOULDER_PITCH, 2580},
                {JointData::ID_L_SHOULDER_ROLL, 2334},
                {JointData::ID_L_ELBOW, 2143},

                {JointData::ID_L_KNEE, 1770},
                {JointData::ID_R_KNEE, 2287}}};

        const ArmPose POSE_CLOSE_GRIPPER = {
            std::map<int, int>{
                {JointData::ID_R_GRIPPER, 2109}}};

        const ArmPose POSE_OPEN_GRIPPER = {
            std::map<int, int>{
                {JointData::ID_R_GRIPPER, 1516}}};

        const ArmPose POSE_ROTATE_WRIST_90DEG = {
            std::map<int, int>{
                {JointData::ID_R_WRIST, 3078}}};

        const ArmPose POSE_ROTATE_WRIST_DEFAULT = {
            std::map<int, int>{
                {JointData::ID_R_WRIST, 2084}}};

        const ArmPose DEFAULT = {
            std::map<int, int>{
                {JointData::ID_R_SHOULDER_PITCH, 1720},
                {JointData::ID_R_SHOULDER_ROLL, 1825},
                {JointData::ID_R_ELBOW, 1716},
                {JointData::ID_R_WRIST, 2118},  // Example: Wrist neutral position (center for 0-4095 range)
                {JointData::ID_R_GRIPPER, 1960} // Example: Gripper open (adjust value as per your gripper's range)
            }};

        void ApplyPose(const ArmPose &pose);
        void SetPID(int p_gain = 4);
    };
}

#endif