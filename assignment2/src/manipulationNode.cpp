#include "utils.h"
#include <exception>
#include <string>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <control_msgs/PointHeadAction.h>
#include <ros/topic.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <assignment2/ArmAction.h>


// bool pickObject(){

// for(int i = 0; i < detection_srv.response.poses.size(); i++)
//         {
//             geometry_msgs::Pose currentPose = detection_srv.response.poses[i];
//             int ids = detection_srv.response.poses_ids[i];
//             float sizes = detection_srv.response.poses_sizes[i];

//         	ROS_INFO("POSITION %d for ID %d", i, ids);
//         	ROS_INFO("X: %f", currentPose.position.x);
//         	ROS_INFO("Y: %f", currentPose.position.y);
//         	ROS_INFO("Z: %f", currentPose.position.z);
//             ROS_INFO("ORIENTATION %d for ID %d", i, ids);
//             ROS_INFO("X: %f", currentPose.orientation.x);
//         	ROS_INFO("Y: %f", currentPose.orientation.y);
//         	ROS_INFO("Z: %f", currentPose.orientation.z);
//             ROS_INFO("W: %f", currentPose.orientation.w);

//             std::pair<geometry_msgs::Pose, int> poseIdPair(currentPose, ids);
//             pairsOfPoses.push_back(poseIdPair);
//         }
//     }
//     else{
//     	ROS_ERROR("Failed to call service to detect object's tag on table");
//     }

// }

class ArmAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<assignment2::ArmAction> as_;
    std::string action_name_;
    assignment2::ArmFeedback feedback_;
    assignment2::ArmResult result_;

public:

    ArmAction(std::string name) : as_(nh_, name, boost::bind(&ArmAction::executeCB, this, _1), false), action_name_(name)
    {
    	as_.start();
    }
    
    ~ArmAction(void){}

    bool pickObject(){
        feedback_.status = 0;
        as_.publishFeedback(feedback_)

        feedback_.status = 1;
        as_.publishFeedback(feedback_);



        return true;
    }

    bool placeObject(){



        return true;
    }


    /**
     * @brief Callback function for executing the navigation action.
     * @param goal The goal for the pose action.
     */
    void executeCB(const assignment2::ArmGoalConstPtr &goal) { 

        switch(goal->request){
            case 1:
                bool objectPicked = pickObject();
                if (objectPicked){
                    result_.arrived = objectPicked;
                    as_.setSucceeded(result_);
                } else {
                    result_.arrived = objectPicked;
                    as_.setAborted(result_);
                }
                break;
            case 2:
                bool objectPlaced = placeObject();
                if (objectPlaced){
                    result_.arrived = objectPlaced;
                    as_.setSucceeded(result_);
                } else {
                    result_.arrived = objectPlaced;
                    as_.setAborted(result_);
                }
                break;
            default:
                ROS_ERROR("Not a possible choice.");
                break;
        }

		


    }
};


int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "manipulationNode");
    ArmAction server("manipulationNode");
    ROS_INFO("Manipulation Server is running...");
    
    ros::spin();

	return 0;
}
