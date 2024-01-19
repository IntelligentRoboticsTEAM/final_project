#include "utils.h"

float degreesToRadians(float degrees) {
    return degrees * (M_PI / 180.0);
}

CartesianCoordinates polarToCartesian(float r, float theta){
    float x = r * cos(theta);
    float y = r * sin(theta);
    return {x, y};
}

std::vector<assignment2::Object> convertToMsgType(const std::vector<Object>& objects) {
    
    std::vector<assignment2::Object> msgObjects;

    for (const Object& object : objects) 
    {
        assignment2::Object msgObject;
        msgObject.x = object.getX();
        msgObject.y = object.getY();
        msgObject.z = object.getZ();
		msgObject.theta = object.getTheta();
		msgObject.dim = object.getDimension();

        msgObjects.push_back(msgObject);
    }
    return msgObjects;
}

std::vector<Object> convertToObjectType(const std::vector<assignment2::Object>& msgObjects) {
    
    std::vector<Object> objects;

    for (int i=0; i < msgObjects.size(); i++) 
    {
        Object obj(msgObjects[i].x, msgObjects[i].y ,msgObjects[i].z , msgObjects[i].theta, msgObjects[i].dim);
        objects.push_back(obj);
    }
    
    return objects;
}
