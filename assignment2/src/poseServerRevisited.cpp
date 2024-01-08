#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <assignment2/PoseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/topic.h>
#include "navigation_methods.h"

/**
 * @brief Defines a class for handling pose actions.
 */
class PoseAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<assignment2::PoseAction> as_;
    std::string action_name_;
    assignment2::PoseFeedback feedback_;
    assignment2::PoseResult result_;

public:
    bool executionDone = false;

    PoseAction(std::string name) : as_(nh_, name, boost::bind(&PoseAction::executeCB, this, _1), false), action_name_(name)
    {
    	as_.start();
    }
    
    ~PoseAction(void){}


    /**
     * @brief Callback function for executing the navigation action.
     * @param goal The goal for the pose action.
     */
    void executeCB(const assignment2::PoseGoalConstPtr &goal) { 

		feedback_.status = 0;
        as_.publishFeedback(feedback_);

		// Initializing data for goalPosition
        Position goalPosition;
        goalPosition.x = goal->x;
        goalPosition.y = goal->y;
        goalPosition.z = goal->z;
        goalPosition.yaw = goal->theta_z;
        ROS_INFO("Goal: x=%f, y=%f, z=%f, theta_z=%f", goalPosition.x, goalPosition.y, goalPosition.z, goalPosition.yaw);

		// Navigation
        feedback_.status = 1;
        as_.publishFeedback(feedback_);
        executionDone = navigateRobotToGoal(goalPosition);

		if(executionDone)
		{
		    feedback_.status = 2; //Sending REACHED_GOAL to feedback
		    as_.publishFeedback(feedback_);
            result_.arrived = executionDone;
		    as_.setSucceeded(result_);
		}else {
            ROS_INFO("Navigation aborted - Timeout reached");
            result_.arrived = executionDone;
            as_.setAborted(result_);
        }
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "poseRevisited");
    PoseAction server("poseRevisited");
    ROS_INFO("Server is running...");
    
    ros::spin();

    return 0;
}
