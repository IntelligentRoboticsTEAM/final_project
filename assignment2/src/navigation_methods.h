#ifndef NAVIGATION_METHODS_H
#define NAVIGATION_METHODS_H

#include <vector>
#include "Obstacle.h"
#include "utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

struct Position {
    float x, y, z;
    float roll, pitch, yaw;
};

enum class StatusRobot : int {
    STOPPED = 0,
    MOVING = 1,
    REACHED_GOAL = 2,
    STARTED_SCAN = 3,
    ENDED_SCAN = 4
};

// declarations
bool navigateRobotToGoal(const Position& goalPosition);
bool navigateRobotToGoal(float x, float y, float z, float theta_z);

#endif // NAVIGATION_METHODS_H