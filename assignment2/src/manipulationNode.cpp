#include "utils.h"
#include <exception>
#include <string>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <control_msgs/PointHeadAction.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/topic.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <apriltag_ros/AprilTagDetectionArray.h>


// bool pickObject(){


// }



int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "manipulation_node");
	ros::NodeHandle n;
	if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
    {
    	ROS_FATAL("Timed-out waiting for valid time.");
    	return EXIT_FAILURE;
  	}
  	
	ROS_INFO("Starting Manipulation application ...");

    // switch(req.request){
    //     case 1:
    //         ros::ServiceServer service = n.advertiseService("/manipulate_object", pickObject);
    //         break;
    //     case 2:
	//         ros::ServiceServer service = n.advertiseService("/manipulate_object", placeObject);
    //         break;
    //     default:
    //         ROS_ERROR("Not a possible choice.");
    //         break;
    // }


	ros::spin();
	
	return 0;
}
