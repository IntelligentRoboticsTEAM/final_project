#include "utils.h"

double degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}

std::pair<float, float> polarToCartesian(float r, float theta){
    float X = r * cos(theta);
    float Y = r * sin(theta);
    
    return std::make_pair(X, Y);
}


// std::pair<float, float> polarToCartesian(float r, float theta){
//     float X = r * cos(theta);
//     float Y = r * sin(theta);
    
//     return std::make_pair(X, Y);
// }

// std::vector<std::vector<float>> clusterRanges(const std::vector<float>& ranges, float th1, int th2) 
// {
//     std::vector<std::vector<float>> clusters;
//     std::vector<float> currentCluster;
//     int last_index = -1;

//     for (int i = 0; i < ranges.size(); i++) {
//     if (std::isfinite(ranges[i])) {
//         if (currentCluster.empty() || (std::abs(ranges[i] - currentCluster.back()) <= th1 &&
//                                     (i - last_index <= th2 || i - last_index <= th2 - ranges.size()))) 
//         {
//             currentCluster.push_back(ranges[i]);
//         } else {
//             clusters.push_back(currentCluster); // Add the current cluster to the list of clusters
//             currentCluster = { ranges[i] }; // Start a new current cluster
//         }

//         last_index = i;
//     }
// }

//     // Checks if beginning and end elements belong to same cluster
//     if (!currentCluster.empty()) {
//         if (std::abs(ranges[0] - currentCluster.back()) <= th1 && (std::abs(0 - last_index) <= th2 || std::abs(0 - last_index) <= th2 - ranges.size())) {
//             currentCluster.insert(currentCluster.end(), ranges.begin(), ranges.begin() + last_index + 1);
//         }
//         clusters.push_back(currentCluster); // adding last cluster
//     }

//     return clusters;
// }

// void computeAvg(const std::vector<std::vector<float>>& rangeClusters,
//                 std::vector<std::pair<float, float>>& output,
//                 float angle_min, float angle_increment) 
// {
    
//     for (int theta = 0; theta < rangeClusters.size(); theta++) 
//     {
//         const auto& cluster = rangeClusters[theta];
//         float sum = 0.0;
//         int count = 0;

//         for (float value : cluster) 
//         {
//             sum += value;
//             ++count;
//         }

//         if (count > 0) 
//         {
//             float average = sum / count;

//             std::pair<float, float> cartesian = polarToCartesian(average, theta);
//             output.push_back(cartesian);
//         }
//     }
//}