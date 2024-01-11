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
        int id = goal.id;

		// Navigation
        feedback_.status = 1;
        as_.publishFeedback(feedback_);

        // 1st waypoint
        navigateRobotToGoal(8.4, 0.0, 0.0, 0.0);

        switch (id) {
            case 1:
                //final position for ID 1
                executionDone = navigateRobotToGoal(8.15, -2.1, 0.0, -110.0);
                break;
            case 2:
                //2nd waypoint
                navigateRobotToGoal(8.40, -4.2, 0.0, 90.0);
                //final position for ID 2
                executionDone = navigateRobotToGoal(7.50, -4.00, 0.0, 70.0);
                break;
            case 3:
                //final position for ID 3
                executionDone = navigateRobotToGoal(7.20, -2.1, 0.0, -50.0);
                break;
            default:
                ROS_ERROR("Error on selecting object ordering");
                break;
        }

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
