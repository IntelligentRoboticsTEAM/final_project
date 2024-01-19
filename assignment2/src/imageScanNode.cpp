#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <assignment2/Scan.h>
#include <geometry_msgs/Pose.h>

const 		std::string window_name = "Cylinder Image";
cv::Mat 	img;
ros::Time 	latestImageStamp;

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
       
    ROS_INFO("Blue: %d, Green: %d, Red: %d", blueCount, greenCount, redCount);
}

std::vector<int> findColorOrder(const cv::Mat &img) {
    std::vector<int> colorOrder;

    int width = img.cols;
    int height = img.rows;
    

    cv::Rect leftRect(0, 0, width / 3, height);                     // (1/3)
    cv::Rect centerRect(width / 3, 0, width / 3, height);           // (1/3)
    cv::Rect rightRect(2 * width / 3, 0, width / 3, height); 		//  (1/3)
    
    
    cv::imshow("Left Region", img(leftRect));
    cv::imshow("Center Region", img(centerRect));
    cv::imshow("Right Region", img(rightRect));
    cv::waitKey(0); 


    // Process left region
    processRegion(img(leftRect), colorOrder);
    // Process center region
    processRegion(img(centerRect), colorOrder);
    // Process right region
    processRegion(img(rightRect), colorOrder);

    return colorOrder;
}


bool resultCB(assignment2::Scan::Request &req, assignment2::Scan::Response &res){
	ROS_INFO("Inside resultCB, before findcolororder");
	std::vector<int> colorOrder = findColorOrder(img);
	ROS_INFO("Inside resultCB, after findcolororder");
    res.ids_associated_colors = colorOrder;
    ROS_INFO("Inside resultCB, response population");
    if(colorOrder.size() > 0)
    	return true;
    else
    	return false;
}


void imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{		
	latestImageStamp = imgMsg->header.stamp;

	cv_bridge::CvImagePtr cvImgPtr;
	try {
		cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

    cvImgPtr->image.copyTo(img);

    cv::imshow(window_name, img);
    cv::waitKey(15);
}


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "image_colors_node");
    
    ros::NodeHandle n;
    
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    
    ros::ServiceServer service = n.advertiseService("/image_colors_node", resultCB);
	
	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
	
	image_transport::ImageTransport it(n);
	image_transport::TransportHints transportHint("compressed");
	image_transport::Subscriber subToImage = it.subscribe("/xtion/rgb/image_raw", 1, imageCallback, transportHint);

	cv::destroyWindow(window_name);
	
    ros::spin();
    return 0;
}
