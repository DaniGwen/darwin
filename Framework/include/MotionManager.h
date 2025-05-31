/*
 *   MotionManager.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _MOTION_MANGER_H_
#define _MOTION_MANGER_H_

#include <list>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include "MotionStatus.h"
#include "MotionModule.h"
#include "CM730.h"
#include "minIni.h"
#include <map>

#define OFFSET_SECTION "Offset"
#define INVALID_VALUE -1024.0

namespace Robot
{
	class MotionManager
	{
	private:
		static MotionManager *m_UniqueInstance;
		std::list<MotionModule *> m_Modules;
		CM730 *m_CM730;
		bool m_ProcessEnable;
		bool m_Enabled;
		int m_FBGyroCenter;
		int m_RLGyroCenter;
		int m_CalibrationStatus;

		bool m_IsRunning;
		bool m_IsThreadRunning;
		bool m_IsLogging;

		std::ofstream m_LogFileStream;
		std::map<int, bool> m_joint_enable_state;

		MotionManager();

	protected:
	public:
		bool DEBUG_PRINT;
		int m_Offset[JointData::NUMBER_OF_JOINTS];

		~MotionManager();

		static MotionManager *GetInstance() { return m_UniqueInstance; }

		bool Initialize(CM730 *cm730);
		bool Reinitialize();
		void Process();
		void SetEnable(bool enable);
		bool GetEnable() { return m_Enabled; }
		void AddModule(MotionModule *module);
		void RemoveModule(MotionModule *module);

		void ResetGyroCalibration()
		{
			m_CalibrationStatus = 0;
			m_FBGyroCenter = 512;
			m_RLGyroCenter = 512;
		}
		int GetCalibrationStatus() { return m_CalibrationStatus; }
		void SetJointDisable(int index);

		void StartLogging();
		void StopLogging();

		void LoadINISettings(minIni *ini);
		void LoadINISettings(minIni *ini, const std::string &section);
		void SaveINISettings(minIni *ini);
		void SaveINISettings(minIni *ini, const std::string &section);

		void SetJointEnableState(int joint_id, bool is_enabled)
		{
			m_joint_enable_state[joint_id] = is_enabled;
		}

		bool IsJointEnabled(int joint_id)
		{
			// Default to true if not found in the map
			if (m_joint_enable_state.find(joint_id) == m_joint_enable_state.end())
			{
				return true;
			}
			return m_joint_enable_state[joint_id];
		}
	};
}

#endif
