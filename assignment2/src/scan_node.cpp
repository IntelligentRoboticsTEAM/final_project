// STD C++ libraries
#include <exception>
#include <string>
// ROS libraries
#include <ros/ros.h>
#include <ros/topic.h>
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>
// ROS messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <image_transport/image_transport.h>
// OpenCV libraries
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// Custom libraries and messages
#include <assignment2/Scan.h>
#include "headers/scan_methods.h"
#include "headers/utils.h"


/*
	*	
	*	GLOBAL VARIABLESS
	*	
*/ 
const 		std::string window_name = "Cylinder Image";
cv::Mat 	img;
ros::Time 	latestImageStamp;


/*
	*	
	*	FUNCTION DECLARATIONS
	*	
*/ 
bool scanResultCB(assignment2::Scan::Request &req, assignment2::Scan::Response &res);
void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg);
void processRegion(const cv::Mat& region, std::vector<int>& colorOrder);
std::vector<int> findColorOrder(const cv::Mat &img);


/*
	*	MAIN
	*	Calls Scan CB
	*	Calls ImageCB
*/ 
int main(int argc, char** argv) {
    
    ros::init(argc, argv, "scan_node");
    
    ros::NodeHandle n;
    
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    
    ros::ServiceServer service = n.advertiseService("/scan_node", scanResultCB);
	
	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
	
	image_transport::ImageTransport it(n);
	image_transport::TransportHints transportHint("compressed");
	image_transport::Subscriber subToImage = it.subscribe("/xtion/rgb/image_raw", 1, imageCallback, transportHint);

	cv::destroyWindow(window_name);
	
    ros::spin();
    return 0;
}


/*
	*	
	*	CALLBACKS
	*	
*/ 

/**
 * @brief This method is used to take the center of the cylinders' positions
 * @param req represents the request of Detection.srv
 * @param res represents the response of Detection.srv
 * @return TRUE if executes correctly and FALSE otherwise.
 */
bool scanResultCB(assignment2::Scan::Request &req, assignment2::Scan::Response &res){
	
	//Get laser info from correct topic
	const std::vector<float> ranges = req.msg.ranges;
	const float angle_increment = req.msg.angle_increment;
	const float angle_min = req.msg.angle_min;
	
	bool flag = false;
	geometry_msgs::Pose pose;
	std::vector<geometry_msgs::Pose> poses;
	
	float th_x = 0.7;
	float th_y = 0.2; 
	
	std::vector<CartesianCoordinates> cartesianRanges = convertRanges(ranges, angle_min, angle_increment);
	std::vector<std::vector<CartesianCoordinates>> cartesianRangesClusters = clusterRanges(cartesianRanges, th_x, th_y);
	poses = findCylinders(cartesianRangesClusters, angle_min, angle_increment);
	
	std::vector<geometry_msgs::Pose> return_vec;
	
	for(int i = 0; i< poses.size(); i++){
		
		//convert poses in odom
		geometry_msgs::PoseStamped in_out_point;
		in_out_point.header.frame_id = "base_laser_link";
		in_out_point.pose = poses[i];
		in_out_point.pose.orientation.x = 0.0;
		in_out_point.pose.orientation.y = 0.0;
		in_out_point.pose.orientation.z = 1.0;
		in_out_point.pose.orientation.w = 0.0;
		
		try
		{
			tf::TransformListener tfListener;
			tfListener.waitForTransform("/base_laser_link", "/map", ros::Time(0), ros::Duration(3.0));
			tfListener.transformPose("/map", in_out_point, in_out_point);
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("Failed to transform point to /map: %s", ex.what());
			return false;
		}
				
		return_vec.push_back(in_out_point.pose);
	}
	
	
	// IMAGE SCAN PART
	std::vector<int> colorOrder = findColorOrder(img);
    
    if(colorOrder.size() > 0 && poses.size() > 0 && colorOrder.size() == poses.size())
    	flag = true;
    else
    	return false;
	
	 
	res.poses = return_vec;
	res.ids_associated_colors = colorOrder;
	
	return flag;
}


/*
	*	
	*	UTILS FUNCTIONS
	*	
*/ 
/**
 * @brief This method is used to return images of the cylinders to compute their colors
 * @param imgMsg gets populated with the images
 */
void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{		
	latestImageStamp = imgMsg->header.stamp;

	cv_bridge::CvImagePtr cvImgPtr;
	try {
		cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

    cvImgPtr->image.copyTo(img);
}

/**
 * @brief This method returns a vector containing the colors of the cylinders in order from right to left
 * @param img variable used to process the three regions of the picture
 * @return the vector of colors represented as int
 */
std::vector<int> findColorOrder(const cv::Mat &img) {
    std::vector<int> colorOrder;

    int width = img.cols;
    int height = img.rows;
    

    cv::Rect leftRect(0, 0, width / 3, height);                     // (1/3)
    cv::Rect centerRect(width / 3, 0, width / 3, height);           // (1/3)
    cv::Rect rightRect(2 * width / 3, 0, width / 3, height); 		// (1/3)
    
	// Process right region
    processRegion(img(rightRect), colorOrder);
    // Process center region
    processRegion(img(centerRect), colorOrder);
    // Process left region
    processRegion(img(leftRect), colorOrder);

    return colorOrder;
}

/**
 * @brief This method divides the picture in three regions and finds the dominant color for each region
 * @param region region of the image to be processed
 * @param colorOrder vector that gets populated with the order of colors
 */
void processRegion(const cv::Mat& region, std::vector<int>& colorOrder) {
    cv::Mat regionRed, regionGreen, regionBlue;
    cv::inRange(region, cv::Scalar(0, 0, 200), cv::Scalar(50, 50, 255), regionRed);
    cv::inRange(region, cv::Scalar(0, 200, 0), cv::Scalar(50, 255, 50), regionGreen);
    cv::inRange(region, cv::Scalar(200, 0, 0), cv::Scalar(255, 50, 50), regionBlue);

    int redCount = cv::countNonZero(regionRed);
    int greenCount = cv::countNonZero(regionGreen);
    int blueCount = cv::countNonZero(regionBlue);

    if (redCount > greenCount && redCount > blueCount)
        colorOrder.push_back(3);
    else if (greenCount > redCount && greenCount > blueCount)
        colorOrder.push_back(2);
    else
        colorOrder.push_back(1);
}


