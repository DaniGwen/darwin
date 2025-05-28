#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <mutex>
#include <map>

// Global mutex for CM730 access
extern std::mutex cm730_mutex;

namespace Robot
{
    struct Pose
    {
        std::map<int, int> joint_positions; // Map of Joint ID to Goal Position Value
    };
}

#endif