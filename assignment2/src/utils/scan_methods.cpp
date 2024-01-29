#include "../headers/scan_methods.h"


/**
 * @brief Converts the coordinates from polar to cartesian for every float number in ranges
 *
 * @param ranges		 	msg.ranges
 * @param angle_min 		Starting angle for the scan, given by /scan topic
 * @param angle_increment 	Increment in the angle, given by /scan topic
 */
std::vector<CartesianCoordinates> convertRanges(const std::vector<float>& ranges, float angle_min, float angle_increment)
{
	ROS_INFO("CONVERT RANGES");
	std::vector<CartesianCoordinates> resultRanges;
	
	 for(int i = 0; i < ranges.size(); i++) 
	 {
	 	CartesianCoordinates currCord = polarToCartesian(ranges[i], angle_min + angle_increment * i); //0
	 	resultRanges.push_back(currCord);
	 }
	 
	ROS_INFO("1) ranges %ld", resultRanges.size()); 
	
	return resultRanges;
}


/**
 * @brief Computes clusters from the ranges fiels of /scan topic message
 * @param ranges The vector ranges
 * @param th1 	 Threshold for accepting two consecutive distances in the same cluster
 */
std::vector<std::vector<CartesianCoordinates>> clusterRanges(const std::vector<CartesianCoordinates> &ranges, float th1, float th2)
{	
	ROS_INFO("CLUSTER RANGES");
	// creation of the structure that contains the clusters
    std::vector<std::vector<CartesianCoordinates>> clusters;
    std::vector<CartesianCoordinates> currentCluster;

    for (int i = 0; i < ranges.size(); i++) {
    	// first condition is to start populate the cluster
    	// second condition: add the next measurement to the cluster if the distance is relatively similar (th1)
    	
    	float x_diff = 0;
    	float y_diff = 0;
    	
    	if(!currentCluster.empty()){
			x_diff = std::abs(ranges[i].x - currentCluster.back().x);	
		 	y_diff = std::abs(ranges[i].y - currentCluster.back().y);
    	}
    
        if (currentCluster.empty() || (x_diff <= th1 && y_diff <= th2) )
        {
            currentCluster.push_back(ranges[i]);
            ROS_INFO("IF CLUSTER[%ld]: ranges[%d].x:%f ranges[%d].y:%f: ", clusters.size(), i, x_diff, i, y_diff);
        } 
        else 
        { // if the difference is too high we move to the next cluster
            ROS_INFO("ELSE CLUSTER[%ld]: ranges[%d].x:%f ranges[%d].y:%f: ", clusters.size(), i, x_diff, i, y_diff);
            clusters.push_back(currentCluster);
            currentCluster = { ranges[i] };
        }
    }
    
    if (!currentCluster.empty())
    	clusters.push_back(currentCluster);
    	
    ROS_INFO("2) clusterRanges %ld", clusters.size()); 

    return clusters;
}


/**
 * @brief Finds the Poses of the three cylinders
 *
 * @param rangeClusters 	Output of the clusterRanges function
 * @param angle_min 		Starting angle for the scan, given by /scan topic
 * @param angle_increment 	Increment in the angle, given by /scan topic
 * @param poses 			Output of this function
 */
std::vector<geometry_msgs::Pose> findCylinders(const std::vector<std::vector<CartesianCoordinates>>& rangeClusters, float angle_min, float angle_increment)
{   

	ROS_INFO("FIND CYLINDER");

	std::vector<geometry_msgs::Pose> poses;
    // step_sizes measures the difference in distance between adjacent measurements of different clusters
	std::vector<float> step_sizes;
	for(int i = 0; i < rangeClusters.size()-1; i++)
	{
		float step = rangeClusters[i].back().x - rangeClusters[i+1][0].x;
		step_sizes.push_back(step);
		ROS_INFO("STEP SIZE for cluster %d\tSTEP: %f", i, step);
	}

	// angle_counter needed for the conversion to cartesian coords
	int angle_counter = 0;
	for(int i = 0; i < rangeClusters.size(); i++)
	{
		std::vector<CartesianCoordinates> currentRange = rangeClusters[i];	
		geometry_msgs::Pose pp;
		
		if(step_sizes[i] > 0 || currentRange.size() < 30 || currentRange.size() > 50) 
		{
			ROS_INFO("IF -- Cluster %d, Size = %ld", i, currentRange.size());
			angle_counter += currentRange.size();
		}
		else {
			ROS_INFO("ELSE -- Cluster %d, Size = %ld", i, currentRange.size());
			//computation of middle point of the Cylinder
			float x_start = currentRange[0].x;
			float y_start = currentRange[0].y;
			
			ROS_INFO("x_start: %f, y_start: %f", x_start, y_start);
			
			angle_counter += currentRange.size();
			
			float x_end = currentRange.back().x;
			float y_end = currentRange.back().y;
			
			ROS_INFO("x_end: %f, y_end: %f", x_end, y_end);

			float x = (x_start + x_end)/2;
			float y = (y_start + y_end)/2;
			
			ROS_INFO("x: %f, y: %f", x, y);

			// adding the Pose to the structure
			pp.position.x = x;
			pp.position.y = y;
			pp.position.z = 0.00;
			
			poses.push_back(pp);	
		}
	}
	
	return poses;
}
