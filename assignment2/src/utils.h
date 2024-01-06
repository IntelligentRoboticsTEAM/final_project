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

double degreesToRadians(double degrees);
std::pair<float, float> polarToCartesian(float r, float theta);
std::vector<assignment2::Object> convertToMsgType(const std::vector<Object>& objects);
std::vector<Object> convertToObjectType(const std::vector<assignment2::Object>& msgObjects);


#endif