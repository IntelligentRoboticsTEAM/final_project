#ifndef SCAN_METHODS_H
#define SCAN_METHODS_H

#include <vector>
#include <cmath>
#include <utility>
#include "Obstacle.h"
#include "utils.h"

std::vector<std::vector<float>> clusterRanges(const std::vector<float>& ranges, float th) ;
std::vector<Obstacle> findObstacles1(const std::vector<std::vector<float>> &rangeClusters, float angle_min, float angle_increment);
std::vector<Obstacle> findObstacles2(const std::vector<std::vector<float>> &rangeClusters, float angle_min, float angle_increment);
std::vector<Obstacle> findObstacles3(const std::vector<std::vector<float>> &rangeClusters, float angle_min, float angle_increment);

#endif // SCAN_METHODS_H
