#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include <cmath>
#include <ir2324_group_10/PoseAction.h>

double degreesToRadians(double degrees);
std::pair<float, float> polarToCartesian(float r, float theta);
std::vector<ir2324_group_10::Obstacle> convertToMsgType(const std::vector<Obstacle>& obstacles);

#endif