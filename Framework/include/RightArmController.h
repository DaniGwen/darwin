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

        Point2D GetHandPositionInCameraView();
        void CenterHandInView(int centering_speed = 50);
        void CloseGripper(int moving_speed = 200, int p_gain = 30);
        void OpenGripper(int moving_speed = 200, int p_gain = 30);
        void RotateWristCW90Deg(int moving_speed = 100, int p_gain = 30);
        void RotateWristCCW90Deg(int moving_speed = 100, int p_gain = 30);
        void HoldItem(int moving_speed = 100, int p_gain = 30);
        void Default();

    private:
        CM730 *cm730_;

        double m_Hand_P_Gain_X; // Gain for correcting horizontal error
        double m_Hand_P_Gain_Y; // Gain for correcting vertical error

        const Pose POSE_CLOSE_GRIPPER = {
            std::map<int, int>{
                {JointData::ID_R_GRIPPER, 2109}}};

        const Pose POSE_OPEN_GRIPPER = {
            std::map<int, int>{
                {JointData::ID_R_GRIPPER, 1516}}};

        const Pose POSE_ROTATE_WRIST_90DEG = {
            std::map<int, int>{
                {JointData::ID_R_WRIST, 3078}}};

        const Pose POSE_ROTATE_WRIST_CCW_90DEG = {
            std::map<int, int>{
                {JointData::ID_R_WRIST, 1299}}};

        const Pose POSE_ROTATE_WRIST_DEFAULT = {
            std::map<int, int>{
                {JointData::ID_R_WRIST, 2084}}};

        const Pose POSE_HOLD_ITEM = {
            std::map<int, int>{
                {JointData::ID_R_SHOULDER_PITCH, 1921},
                {JointData::ID_R_SHOULDER_ROLL, 2023},
                {JointData::ID_R_ELBOW, 2652},
                {JointData::ID_R_WRIST, 3100},
                {JointData::ID_R_GRIPPER, 2109}}};

        const Pose DEFAULT = {
            std::map<int, int>{
                {JointData::ID_R_SHOULDER_PITCH, 1719},
                {JointData::ID_R_SHOULDER_ROLL, 2025},
                {JointData::ID_R_ELBOW, 1747}}};

        void ApplyPose(const Pose &pose, int speed = 200);
        void SetPID(int p_gain = 30);
    };
}

#endif