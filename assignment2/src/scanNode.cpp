// STD C++ libraries
#include <exception>
#include <string>

// ROS libraries
#include <ros/ros.h>
#include <ros/topic.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <control_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/shared_ptr.hpp>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

// OpenCV libraries
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// Our libraries
#include <assignment2/Scan.h>  
#include "headers/scan_methods.h"
#include "headers/utils.h"


bool scanCB(assignment2::Scan::Request &req, assignment2::Scan::Response &res){

	ROS_INFO("Incoming request: %s", req.ready ? "true" : "false");
	
	//Get laser info from correct topic
	const std::vector<float> ranges = req.msg.ranges;
	const float angle_increment = req.msg.angle_increment;
	const float angle_min = req.msg.angle_min;

	
	geometry_msgs::Pose pose;
	std::vector<geometry_msgs::Pose> poses;
	
	float th_x = 0.7;
	float th_y = 0.2; 	// PRIMA 0.7
	
	std::vector<CartesianCoordinates> cartesianRanges = convertRanges(ranges, angle_min, angle_increment);
	std::vector<std::vector<CartesianCoordinates>> cartesianRangesClusters = clusterRanges(cartesianRanges, th_x, th_y);
	poses = findCylinders(cartesianRangesClusters, angle_min, angle_increment);
	
	std::vector<geometry_msgs::Pose> return_vec;
	
	for(int i = 0; i< poses.size(); i++){
		ROS_INFO("Poses[%d] in /base_laser_link= x: %f, y: %f, z: %f", i, poses[i].position.x, poses[i].position.y, poses[i].position.x);
		
		//convert poses in odom
		geometry_msgs::PoseStamped in_out_point;
		in_out_point.header.frame_id = "base_laser_link"; //da rivedere
		in_out_point.pose = poses[i];
		in_out_point.pose.orientation.x = 0.0;
		in_out_point.pose.orientation.y = 0.0;
		in_out_point.pose.orientation.z = 1.0;
		in_out_point.pose.orientation.w = 0.0;
		
		try
		{
			tf::TransformListener tfListener;
			tfListener.waitForTransform("/base_laser_link", "/odom", ros::Time(0), ros::Duration(3.0));
			tfListener.transformPose("/odom", in_out_point, in_out_point);
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("Failed to transform point to /odom: %s", ex.what());
			return 1;
		}
		
		ROS_INFO("Poses[%d] in /odom = x: %f, y: %f, z: %f", i, in_out_point.pose.position.x, in_out_point.pose.position.y, in_out_point.pose.position.x);
		
		return_vec.push_back(in_out_point.pose);
	}
	
	 
	res.poses = return_vec;
	
	
	//res.poses = poses;
	
	return true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan_node");
	ros::NodeHandle n;
	
	if (!ros::Time::waitForValid(ros::WallDuration(10.0))) {ROS_FATAL("Timed-out waiting for valid time."); 	return EXIT_FAILURE; }
	ROS_INFO("Starting Scan of cylinders ...");
	
	ros::ServiceServer service = n.advertiseService("/scan_node", scanCB);
	ros::spin();
	
	return 0;
}
