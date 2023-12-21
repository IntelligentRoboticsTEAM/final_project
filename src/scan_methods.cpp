#include "scan_methods.h"

std::vector<std::vector<float>> clusterRanges(const std::vector<float>& ranges, float th1) 
{
    std::vector<std::vector<float>> clusters;
    std::vector<float> currentCluster;
    int last_index = -1;

    for (int i = 0; i < ranges.size(); i++) {
        if (currentCluster.empty() || (std::abs(ranges[i] - currentCluster.back()) <= th1 )) 
        {
            currentCluster.push_back(ranges[i]);
        } else {
            clusters.push_back(currentCluster);
            currentCluster = { ranges[i] }; 
        }

        last_index = i;
    }

    return clusters;
}

std::vector<Obstacle> computeAvg(const std::vector<std::vector<float>>& rangeClusters,
                                 float angle_min, float angle_increment) 
{
    std::vector<Obstacle> obstacles;
    
    for (int theta = 0; theta < rangeClusters.size(); theta++) 
    {
        const auto& cluster = rangeClusters[theta];
        float sum = 0.0;
        int count = 0;

        for (float value : cluster) 
        {
            sum += value;
            ++count;
        }

        if (count > 0) 
        {
            float average = sum / count;
            std::pair<float, float> cartesian = polarToCartesian(average, theta);

            Obstacle obs(cartesian.first, cartesian.second, count);

            obstacles.push_back(obs);
        }
    }
    return obstacles;
}