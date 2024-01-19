#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include <cmath>
#include "assignment2/Detection.h"
#include "assignment2/Object.h"
#include "Object.h"

enum class Status {
    PickStarted = 0,
    PlaceStarted = 1,
    HandOpen = 2,
    HandClosed = 3,
    ArmHigh = 4,
    ArmLow = 5,
    ActionEnded = 6
};

struct CartesianCoordinates {
    float x;
    float y;
};

struct PoseID{
	CartesianCoordinates pose;
	int id;
};

float degreesToRadians(float degrees);
CartesianCoordinates polarToCartesian(float r, float theta);
std::vector<assignment2::Object> convertToMsgType(const std::vector<Object>& objects);
std::vector<Object> convertToObjectType(const std::vector<assignment2::Object>& msgObjects);

#endif // UTILS_H
