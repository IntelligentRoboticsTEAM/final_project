#include "scan_methods.h"
#include <math.h>

std::vector<std::vector<float>> clusterRanges(const std::vector<float>& ranges, float th1)
{	
    std::vector<std::vector<float>> clusters;
    std::vector<float> currentCluster;
    int start_index = 0;

    for (int i = 0; i < ranges.size(); i++) {
        if (currentCluster.empty() || (std::abs(ranges[i] - currentCluster.back()) <= th1 ))
        {
            currentCluster.push_back(ranges[i]);
        } else {
            clusters.push_back(currentCluster);
            currentCluster = { ranges[i] };
            start_index = i+1;    
        }
    }
    
    return clusters;
}


std::vector<Obstacle> findObstacles1(const std::vector<std::vector<float>>& rangeClusters, float angle_min, float angle_increment)
{
    std::vector<Obstacle> obstacles;
	
	std::vector<double> step_sizes;
	for(int i = 0; i < rangeClusters.size()-1; i++)
	{
		double step = rangeClusters[i].back() - rangeClusters[i+1][0];
		step_sizes.push_back(step);
	}

	int angle_counter = 0;
	for(int i = 0; i < rangeClusters.size(); i++)
	{
		if(step_sizes[i] > 0 || rangeClusters[i].size() < 15 || rangeClusters[i].size() > 100) 
		{
			angle_counter += rangeClusters[i].size();
		}else
		{
			double x_start = rangeClusters[i][0] * cos(angle_min + angle_increment*angle_counter);
			double y_start = rangeClusters[i][0] * sin(angle_min + angle_increment*angle_counter);
			
			angle_counter += rangeClusters[i].size();
			
			double x_end = rangeClusters[i].back() * cos(angle_min + angle_increment*(angle_counter-1));
			double y_end = rangeClusters[i].back() * sin(angle_min + angle_increment*(angle_counter-1));

			double x = (x_start + x_end)/2;
			double y = (y_start + y_end)/2;

			Obstacle o(x, y, i);
			obstacles.push_back(o);
			
		}
	}

    return obstacles;
}

std::vector<Obstacle> findObstacles2(const std::vector<std::vector<float>> &rangeClusters, float angle_min, float angle_increment)
{
 
    int angle_counter = 0;
    std::vector<Obstacle> obstacles;

    for (int i = 0; i < rangeClusters.size(); i++)
    {
 
        std::vector<float> currVec = rangeClusters[i];
        std::vector<float> coefficients;
 
        for (int j = 0; j < currVec.size() - 1; j++)
        {
 
            float m = 0;
            double y2 = currVec[j + 1] * sin(angle_min + angle_increment * (angle_counter + (j + 1)));
            double y1 = currVec[j] * sin(angle_min + angle_increment * (angle_counter + j));
 
            double x2 = currVec[j + 1] * cos(angle_min + angle_increment * (angle_counter + (j + 1)));
            double x1 = currVec[j] * cos(angle_min + angle_increment * (angle_counter + j));
 
            m = (y2 - y1) / (x2 - x1);
 
            coefficients.push_back(m);
        }
 
        int count = 0;
        for (int k = 0; k < coefficients.size() - 1; k++)
        {
            if (std::abs(coefficients[k + 1] - coefficients[k]) > 0.1)
            {
                count++;
            }
        }
        if (count > 0.5 * coefficients.size())
        {
 
            int indexHalfVector = static_cast<int>(currVec.size() / 2);
 
            double x = currVec[indexHalfVector] * cos(angle_min + angle_increment * (angle_counter + indexHalfVector));
            double y = currVec[indexHalfVector] * sin(angle_min + angle_increment * (angle_counter + indexHalfVector));
 
            Obstacle o(x, y, i);
            obstacles.push_back(o);
        }
 
        angle_counter += currVec.size();
    }
    return obstacles;
}

std::vector<Obstacle> findObstacles3(const std::vector<std::vector<float>> &rangeClusters, float angle_min, float angle_increment)
{
 
    int angle_counter = 0;
    std::vector<Obstacle> obstacles;
 
    for (int i = 0; i < rangeClusters.size(); i++)
    {
 
        std::vector<float> currVec = rangeClusters[i];
        std::vector<float> coefficients;
        if(currVec.size() < 15 || currVec.size() > 70) continue; // condizione per prevenire "spiragli"
 
        for (int j = 0; j < currVec.size() - 1; j++)
        {
 
            float m = 0;
            double y2 = currVec[j + 1] * sin(angle_min + angle_increment * (angle_counter + (j + 1)));
            double y1 = currVec[j] * sin(angle_min + angle_increment * (angle_counter + j));
 
            double x2 = currVec[j + 1] * cos(angle_min + angle_increment * (angle_counter + (j + 1)));
            double x1 = currVec[j] * cos(angle_min + angle_increment * (angle_counter + j));
 			
 			if(x1 == x2) m = 1000000;
 			else m = (y2 - y1) / (x2 - x1);
 
            coefficients.push_back(m);
        }
        
        double coeff_sum = 0;
        for(int i = 0; i < coefficients.size(); i++) coeff_sum += coefficients[i];
 
        if (abs(coeff_sum) < 20)
        {
 
            int indexHalfVector = static_cast<int>(currVec.size() / 2);
 
            double x = currVec[indexHalfVector] * cos(angle_min + angle_increment * (angle_counter + indexHalfVector));
            double y = currVec[indexHalfVector] * sin(angle_min + angle_increment * (angle_counter + indexHalfVector));
 
            Obstacle o(x, y, i);
            obstacles.push_back(o);
        }
 
        angle_counter += currVec.size();
        
    }
    return obstacles;
}
        
