#include "utils.h"

double degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}

std::pair<float, float> polarToCartesian(float r, float theta){
    float X = r * cos(theta);
    float Y = r * sin(theta);
    
    return std::make_pair(X, Y);
}

