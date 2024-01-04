#ifndef OBJECT_H
#define OBJECT_H

#include <iostream>

class Object {
private:
    float x;
    float y;
    float z;
    float theta;
    float dim;

public:
    // Constructor
    Object(float x, float y, float z, float theta, float dim) : x(x), y(y), z(z),
                                                                theta(theta), dim(dim) {}

    // Getters
    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }
    float getTheta() const { return theta; }
    float getDimension() const { return dim; }
};

#endif // OBJECT_H
