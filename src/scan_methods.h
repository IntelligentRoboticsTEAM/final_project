#ifndef SCAN_METHODS_H
#define SCAN_METHODS_H

#include <vector>
#include <cmath>
#include <utility>
#include "Obstacle.h"
#include "utils.h"

std::vector<Obstacle> computeAvg(const std::vector<std::vector<float>>& rangeClusters,
                                 float angle_min, float angle_increment);
std::vector<std::vector<float>> clusterRanges(const std::vector<float>& ranges, float th) ;

#endif // SCAN_METHODS_H
