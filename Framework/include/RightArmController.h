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

        void HandReach(int p_gain = 4);
        void CloseGripper(int p_gain = 4);
        void OpenGripper(int p_gain = 4);
        void RotateWrist90Deg(int p_gain = 4);
        void Default();

    private:
        CM730 *cm730_;

        // Define poses for the right arm including wrist and gripper
        // Assuming JointData::ID_R_WRIST = 21 and JointData::ID_R_GRIPPER = 22 are defined in JointData.h
        // Placeholder values for wrist and gripper have been added. Adjust as needed.
        const Pose POSE_REACH_HAND = {
            std::map<int, int>{
                {JointData::ID_R_SHOULDER_PITCH, 2376},
                {JointData::ID_R_SHOULDER_ROLL, 1749},
                {JointData::ID_R_ELBOW, 1628},

                {JointData::ID_L_SHOULDER_PITCH, 2580},
                {JointData::ID_L_SHOULDER_ROLL, 2334},
                {JointData::ID_L_ELBOW, 2143}}};

        const Pose POSE_CLOSE_GRIPPER = {
            std::map<int, int>{
                {JointData::ID_R_GRIPPER, 2109}}};

        const Pose POSE_OPEN_GRIPPER = {
            std::map<int, int>{
                {JointData::ID_R_GRIPPER, 1516}}};

        const Pose POSE_ROTATE_WRIST_90DEG = {
            std::map<int, int>{
                {JointData::ID_R_WRIST, 3078}}};

        const Pose POSE_ROTATE_WRIST_DEFAULT = {
            std::map<int, int>{
                {JointData::ID_R_WRIST, 2084}}};

        const Pose DEFAULT = {
            std::map<int, int>{
                {JointData::ID_R_SHOULDER_PITCH, 1720},
                {JointData::ID_R_SHOULDER_ROLL, 1825},
                {JointData::ID_R_ELBOW, 1716}}};

        void ApplyPose(const Pose &pose);
        void SetPID(int p_gain = 4);
    };
}

#endif