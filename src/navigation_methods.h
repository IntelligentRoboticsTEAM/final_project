#ifndef NAVIGATION_METHODS_H
#define NAVIGATION_METHODS_H

#include <vector>
#include "Obstacle.h"

struct Position {
    float x;
    float y;
    float z;
};

// declarations
bool navigateRobotToGoal(const Position& goalPosition);
int scanObstacles(std::vector<Obstacle>& obstacles);

#endif // NAVIGATION_METHODS_H
