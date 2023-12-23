#include "utils.h"

double degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}

std::pair<float, float> polarToCartesian(float r, float theta){
    float X = r * cos(theta);
    float Y = r * sin(theta);
    
    return std::make_pair(X, Y);
}

/**
     * @brief Convert std::vector<Obstacle> to ir2324_group_10::Obstacle[] (--> message type)
     * @param obstacles The obstacles to convert.
     * @return Vector of obstacles in message format.
     */
    std::vector<ir2324_group_10::Obstacle> convertToMsgType(const std::vector<Obstacle>& obstacles) {
        std::vector<ir2324_group_10::Obstacle> msgObstacles;

        for (const Obstacle& obstacle : obstacles) 
        {
            ir2324_group_10::Obstacle msgObstacle;
            msgObstacle.x = obstacle.getX();
            msgObstacle.y = obstacle.getY();
            msgObstacle.radius = obstacle.getRadius();

            msgObstacles.push_back(msgObstacle);
        }

        return msgObstacles;
    }
