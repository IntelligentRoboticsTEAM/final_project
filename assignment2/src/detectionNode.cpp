#include "utils.h"
#include <exception>
#include <string>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <control_msgs/PointHeadAction.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/topic.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

ros::Time latestImageStamp;

// ROS call back for every new image received
void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
		
	latestImageStamp = imgMsg->header.stamp;

	cv_bridge::CvImagePtr cvImgPtr;
	cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
	
	cv::imshow("Inside of TIAGo's head", cvImgPtr->image);
	cv::waitKey(15);


bool lookToPoint(assignment2::Detection::Request &req, assignment2::Detection::Response &res){

	ROS_INFO("Incoming request: %s", req.ready ? "true" : "false");
	
	//simple action client to interact with tiago's head
	actionlib::SimpleActionClient<control_msgs::PointHeadAction> pointHeadClient("/head_controller/point_head_action", true); 
	
	int iterations = 0, max_iterations = 3;
	// Wait for head controller action server to come up
	while(!pointHeadClient.waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations)
	{
		ROS_INFO("Waiting for the point_head_action server to come up");
		++iterations;
	}
	if(iterations == max_iterations)
	    ROS_ERROR("Error in create PointHeadClient: head controller action server not available");

	//Get camera info from correct topic
	sensor_msgs::CameraInfoConstPtr msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/xtion/rgb/camera_info", ros::Duration(10.0));
	
	//define a cv::Mat to store the image
	cv::Mat cameraIntrinsics;
 	if(msg.use_count() > 0)
	{
		cameraIntrinsics = cv::Mat::zeros(3,3,CV_64F);
		cameraIntrinsics.at<double>(0, 0) = msg->K[0]; //fx
		cameraIntrinsics.at<double>(1, 1) = msg->K[4]; //fy
		cameraIntrinsics.at<double>(0, 2) = msg->K[2]; //cx
		cameraIntrinsics.at<double>(1, 2) = msg->K[5]; //cy
		cameraIntrinsics.at<double>(2, 2) = 1;
	}
	
	//define a target point to make tiago point at
	geometry_msgs::PointStamped pointStamped; 
	
	pointStamped.header.frame_id = "/xtion_rgb_optical_frame"; 
	pointStamped.header.stamp    = latestImageStamp;
	pointStamped.point.x = 0.11;
	pointStamped.point.y = 0.38;
	pointStamped.point.z = 1.00;  
	
	/*
	tf::TransformListener tfListener;
	try
    {
        tfListener.waitForTransform("/xtion_rgb_optical_frame", pointStamped.header.frame_id, ros::Time(0), ros::Duration(3.0));
        tfListener.transformPoint("/xtion_rgb_optical_frame", pointStamped, pointStamped);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("Failed to transform table point to /xtion_rgb_optical_frame: %s", ex.what());
        return 1;
    } 
    */

	ROS_INFO("X: %f, Y: %f, Z: %f", (float)pointStamped.point.x, (float)pointStamped.point.y, (float)pointStamped.point.z);

	//the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
	control_msgs::PointHeadGoal goal;
	
	goal.pointing_frame = "/xtion_rgb_optical_frame";
	goal.pointing_axis.x = 0.0;
	goal.pointing_axis.y = 0.0;
	goal.pointing_axis.z = 1.0;
	goal.min_duration = ros::Duration(1.0);
	goal.max_velocity = 0.25;
	goal.target = pointStamped;

	pointHeadClient.sendGoal(goal);
	
	bool finished_before_timeout = pointHeadClient.waitForResult(ros::Duration(20.0));
 
	//print result
    if(finished_before_timeout) {
        actionlib::SimpleClientGoalState state = pointHeadClient.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("I'm pointing to the given point");  
        }
    }else{
        ROS_INFO("Pointing did not finish before the timeout.");
        pointHeadClient.cancelGoal();
        ROS_INFO("Pointing goal has been cancelled");
    }
	
	std::vector<Object> objects;
	
	Object o_1(1,1,1,90,0.5);
	Object o_2(2,2,2,180,0.7);
	
	objects.push_back(o_1);
	objects.push_back(o_2);
	
	std::vector<assignment2::Object> msgObjects = convertToMsgType(objects);
	
	res.objects_pose = msgObjects;

	return true;
	
}

bool scanQR (){





}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Entry point
int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "detection_node");
	ros::NodeHandle n;
	if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
    {
    	ROS_FATAL("Timed-out waiting for valid time.");
    	return EXIT_FAILURE;
  	}
  	
	ROS_INFO("Starting QR pose detection application ...");
	
	ros::ServiceServer service = n.advertiseService("/object_detection", lookToPoint);

	// Create the window to show TIAGo's camera images
	cv::namedWindow("Inside of TIAGo's head", cv::WINDOW_AUTOSIZE);
	
	// Define ROS topic from where TIAGo publishes images
	image_transport::ImageTransport it(n);
	// use compressed image transport to use less network bandwidth
	image_transport::TransportHints transportHint("compressed");

	//ROS_INFO_STREAM("Subscribing to " << imageTopic << " ...");
	image_transport::Subscriber subToImage = it.subscribe("/xtion/rgb/image_raw", 1, imageCallback, transportHint);
	
	ros::Subscriber subToTags = nh.subscribe("/tag_detections",1 , scanQR);

	cv::destroyWindow("Inside of TIAGo's head");
	
	ros::spin();
	
	return 0;
}
