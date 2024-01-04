#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <iostream>

class Obstacle {
private:
    float x;
    float y;
    float radius;

public:
    // Constructor
    Obstacle(float x, float y, float radius) : x(x), y(y), radius(radius) {}

    // Getters
    float getX() const { return x; }
    float getY() const { return y; }
    float getRadius() const { return radius; }
};

#endif // OBSTACLE_H
