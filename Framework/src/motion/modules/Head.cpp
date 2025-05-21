/*
 * Head.cpp
 *
 * Author: ROBOTIS
 *
 */

#include <stdio.h>
#include <iostream> // For std::cout, std::cerr, std::endl
#include "MX28.h"
#include "Kinematics.h"
#include "MotionStatus.h"
#include "Head.h"


using namespace Robot;


Head* Head::m_UniqueInstance = new Head();

Head::Head()
{
    m_Pan_p_gain = 0.1;
    m_Pan_d_gain = 0.22;

    m_Tilt_p_gain = 0.1;
    m_Tilt_d_gain = 0.22;

    m_LeftLimit = 70;
    m_RightLimit = -70;
    m_TopLimit = Kinematics::EYE_TILT_OFFSET_ANGLE;
    m_BottomLimit = Kinematics::EYE_TILT_OFFSET_ANGLE - 65;

    m_Pan_Home = 0.0;
    m_Tilt_Home = Kinematics::EYE_TILT_OFFSET_ANGLE - 30.0;

    m_Joint.SetEnableHeadOnly(true); // This should be handled by HeadTracking::Initialize
}

Head::~Head()
{
}

void Head::CheckLimit()
{
    if(m_PanAngle > m_LeftLimit)
        m_PanAngle = m_LeftLimit;
    else if(m_PanAngle < m_RightLimit)
        m_PanAngle = m_RightLimit;

    if(m_TiltAngle > m_TopLimit)
        m_TiltAngle = m_TopLimit;
    else if(m_TiltAngle < m_BottomLimit)
        m_TiltAngle = m_BottomLimit;

    // DEBUG: Print angles after limit check
    // std::cout << "DEBUG: Head::CheckLimit - PanAngle: " << m_PanAngle << ", TiltAngle: " << m_TiltAngle << std::endl;
}

void Head::LoadINISettings(minIni* ini)
{
    m_Pan_p_gain = ini->getd("Head Pan/Tilt", "Pan_P_GAIN", m_Pan_p_gain);
    m_Pan_d_gain = ini->getd("Head Pan/Tilt", "Pan_D_GAIN", m_Pan_d_gain);
    m_Tilt_p_gain = ini->getd("Head Pan/Tilt", "Tilt_P_GAIN", m_Tilt_p_gain);
    m_Tilt_d_gain = ini->getd("Head Pan/Tilt", "Tilt_D_GAIN", m_Tilt_d_gain);

    m_LeftLimit = ini->getd("Head Pan/Tilt", "LeftLimit", m_LeftLimit);
    m_RightLimit = ini->getd("Head Pan/Tilt", "RightLimit", m_RightLimit);
    m_TopLimit = ini->getd("Head Pan/Tilt", "TopLimit", m_TopLimit);
    m_BottomLimit = ini->getd("Head Pan/Tilt", "BottomLimit", m_BottomLimit);

    m_Pan_Home = ini->getd("Head Pan/Tilt", "Pan_Home", m_Pan_Home);
    m_Tilt_Home = ini->getd("Head Pan/Tilt", "Tilt_Home", m_Tilt_Home);

    std::cout << "INFO: Head::LoadINISettings - Pan_P_GAIN: " << m_Pan_p_gain
              << ", Pan_D_GAIN: " << m_Pan_d_gain
              << ", Tilt_P_GAIN: " << m_Tilt_p_gain
              << ", Tilt_D_GAIN: " << m_Tilt_d_gain << std::endl;
    std::cout << "INFO: Head::LoadINISettings - Limits: L=" << m_LeftLimit << ", R=" << m_RightLimit
              << ", T=" << m_TopLimit << ", B=" << m_BottomLimit << std::endl;
    std::cout << "INFO: Head::LoadINISettings - Home: Pan=" << m_Pan_Home << ", Tilt=" << m_Tilt_Home << std::endl;
}

void Head::MoveToHome()
{
    MoveByAngle(m_Pan_Home, m_Tilt_Home);
    std::cout << "DEBUG: Head::MoveToHome - Moving to Pan_Home: " << m_Pan_Home << ", Tilt_Home: " << m_Tilt_Home << std::endl;
}

void Head::MoveByAngle(double pan, double tilt)
{
    m_PanAngle = pan;
    m_TiltAngle = tilt;

    CheckLimit();
    std::cout << "DEBUG: Head::MoveByAngle - Target Pan: " << pan << ", Target Tilt: " << tilt
              << " (After Limit: Pan: " << m_PanAngle << ", Tilt: " << m_TiltAngle << ")" << std::endl;
}

void Head::MoveByAngleOffset(double pan, double tilt)
{
    MoveByAngle(m_PanAngle + pan, m_TiltAngle + tilt);
    std::cout << "DEBUG: Head::MoveByAngleOffset - Offset Pan: " << pan << ", Offset Tilt: " << tilt << std::endl;
}

void Head::InitTracking()
{
    m_Pan_err = 0;
    m_Pan_err_diff = 0;
    m_Tilt_err = 0;
    m_Tilt_err_diff = 0;
    std::cout << "DEBUG: Head::InitTracking - Tracking errors reset." << std::endl;
}

void Head::MoveTracking(Point2D err)
{
    m_Pan_err_diff = err.X - m_Pan_err;
    m_Pan_err = err.X;

    m_Tilt_err_diff = err.Y - m_Tilt_err;
    m_Tilt_err = err.Y;

    std::cout << "DEBUG: Head::MoveTracking(Point2D err) - Input P_err.X: " << err.X << ", P_err.Y: " << err.Y << std::endl;
    std::cout << "DEBUG: Head::MoveTracking(Point2D err) - m_Pan_err: " << m_Pan_err << ", m_Pan_err_diff: " << m_Pan_err_diff << std::endl;
    std::cout << "DEBUG: Head::MoveTracking(Point2D err) - m_Tilt_err: " << m_Tilt_err << ", m_Tilt_err_diff: " << m_Tilt_err_diff << std::endl;

    MoveTracking(); // Call the overloaded version
}

void Head::MoveTracking()
{
    double pOffset, dOffset;

    // Pan PID calculation
    pOffset = m_Pan_err * m_Pan_p_gain;
    dOffset = m_Pan_err_diff * m_Pan_d_gain;
    m_PanAngle += (pOffset + dOffset);

    // Tilt PID calculation
    pOffset = m_Tilt_err * m_Tilt_p_gain;
    dOffset = m_Tilt_err_diff * m_Tilt_d_gain;
    m_TiltAngle += (pOffset + dOffset);

    std::cout << "DEBUG: Head::MoveTracking() - Pan pOffset: " << m_Pan_err * m_Pan_p_gain
              << ", Pan dOffset: " << m_Pan_err_diff * m_Pan_d_gain
              << ", New PanAngle (before limit): " << m_PanAngle << std::endl;
    std::cout << "DEBUG: Head::MoveTracking() - Tilt pOffset: " << m_Tilt_err * m_Tilt_p_gain
              << ", Tilt dOffset: " << m_Tilt_err_diff * m_Tilt_d_gain
              << ", New TiltAngle (before limit): " << m_TiltAngle << std::endl;

    CheckLimit(); // Apply limits after calculating new angles
}

void Head::Process()
{
    // Check if Head joints are enabled by checking individual joint enable status
    // This assumes JointData::GetEnable(id) is available and returns true if enabled.
    if(m_Joint.GetEnable(JointData::ID_HEAD_PAN) && m_Joint.GetEnable(JointData::ID_HEAD_TILT)){
        // Set target angles for the head motors
        m_Joint.SetAngle(JointData::ID_HEAD_PAN, m_PanAngle);
        m_Joint.SetAngle(JointData::ID_HEAD_TILT, m_TiltAngle);

        std::cout << "DEBUG: Head::Process - Setting Pan Angle: " << m_PanAngle
                  << ", Tilt Angle: " << m_TiltAngle << std::endl;
    } else {
        std::cout << "DEBUG: Head::Process - Head joints are not enabled, skipping angle setting." << std::endl;
    }
}
