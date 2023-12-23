#include "scan_methods.h"
#include <math.h>

/**
 * @brief Computes clusters from the ranges fiels of /scan topic message
 * @param ranges The vector ranges
 * @param th1 Threshold for accepting two consecutive distances in the same cluster
 */
std::vector<std::vector<float>> clusterRanges(const std::vector<float>& ranges, float th1)
{	
	// creation of the structure that contains the clusters
    std::vector<std::vector<float>> clusters;
    std::vector<float> currentCluster;

    for (int i = 0; i < ranges.size(); i++) {
    	// first condition is to start populate the cluster
    	// second condition: add the next measurement to the cluster if the distance is relatively similar (th1)
        if (currentCluster.empty() || (std::abs(ranges[i] - currentCluster.back()) <= th1 ))
        {
            currentCluster.push_back(ranges[i]);
        } else {
        	// if the difference is too high we move to the next cluster
            clusters.push_back(currentCluster);
            currentCluster = { ranges[i] };
        }
    }
    
    return clusters;
}

/**
 * @brief Finds obstacles
 * @param rangeClusters Output of the clusterRanges function
 * @param angle_min Starting angle for the scan, given by /scan topic
 * @param angle_increment Increment in the angle, given by /scan topic
 */
std::vector<Obstacle> findObstacles1(const std::vector<std::vector<float>>& rangeClusters, float angle_min, float angle_increment)
{
	// structure that stores the obstacles
    std::vector<Obstacle> obstacles;
    
    // step_sizes measures the difference in distance between adjacent measurements of different clusters
	std::vector<double> step_sizes;
	for(int i = 0; i < rangeClusters.size()-1; i++)
	{
		double step = rangeClusters[i].back() - rangeClusters[i+1][0];
		step_sizes.push_back(step);
	}

	// angle_counter needed for the conversion to cartesian coords
	int angle_counter = 0;
	for(int i = 0; i < rangeClusters.size(); i++)
	{
		if(step_sizes[i] > 0 || rangeClusters[i].size() < 15 || rangeClusters[i].size() > 100) 
		{
			angle_counter += rangeClusters[i].size();
		}else
		{
		
			//computation of middle point of the obstacle
			double x_start = rangeClusters[i][0] * cos(angle_min + angle_increment*angle_counter);
			double y_start = rangeClusters[i][0] * sin(angle_min + angle_increment*angle_counter);
			
			angle_counter += rangeClusters[i].size();
			
			double x_end = rangeClusters[i].back() * cos(angle_min + angle_increment*(angle_counter-1));
			double y_end = rangeClusters[i].back() * sin(angle_min + angle_increment*(angle_counter-1));

			double x = (x_start + x_end)/2;
			double y = (y_start + y_end)/2;

			// adding the obstacle to the structure
			Obstacle o(x, y, i);
			obstacles.push_back(o);
			
		}
	}

    return obstacles;
}

/**
 * @brief Finds obstacles
 * @param rangeClusters Output of the clusterRanges function
 * @param angle_min Starting angle for the scan, given by /scan topic
 * @param angle_increment Increment in the angle, given by /scan topic
 */
std::vector<Obstacle> findObstacles2(const std::vector<std::vector<float>> &rangeClusters, float angle_min, float angle_increment)
{
 
    int angle_counter = 0;
    std::vector<Obstacle> obstacles;

    for (int i = 0; i < rangeClusters.size(); i++)
    {

		// computation of the angular coeffs (stored in coeffiecients vector) 
        std::vector<float> currVec = rangeClusters[i];
        std::vector<float> coefficients;
 
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

		// we count the number of times difference of adjiacent angular coeffs is >0.1 (because we are taking into consideration the loss of precision)
		//  if that happens too many times, it means we are not looking at a wall (which is mostly linear)
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

/**
 * @brief Finds obstacles
 * @param rangeClusters Output of the clusterRanges function
 * @param angle_min Starting angle for the scan, given by /scan topic
 * @param angle_increment Increment in the angle, given by /scan topic
 */
std::vector<Obstacle> findObstacles3(const std::vector<std::vector<float>> &rangeClusters, float angle_min, float angle_increment)
{
 
    int angle_counter = 0;
    std::vector<Obstacle> obstacles;
 
    for (int i = 0; i < rangeClusters.size(); i++)
    {
 	
 		// computation of the angular coeffs (stored in coeffiecients vector) 
        std::vector<float> currVec = rangeClusters[i];
        std::vector<float> coefficients;
        
        // condition to avoid the detection of small gaps between objects as objects themselves
        if(currVec.size() < 15 || currVec.size() > 70) continue; 
 
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
        
        // contains the sum of all angular coeffs of a cluster
        double coeff_sum = 0;
        for(int i = 0; i < coefficients.size(); i++) coeff_sum += coefficients[i];
 
 		// coeff_sum should be ~0 if we are detecting a semi-circle, so we set a low threshold
        if (abs(coeff_sum) < 5)
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
        
