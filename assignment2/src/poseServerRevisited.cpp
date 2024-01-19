#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <assignment2/PoseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/topic.h>
#include "navigation_methods.h"
#include <geometry_msgs/Pose.h>
#include <apriltag_ros/AprilTagDetection.h>
/**
 * @brief Defines a class for handling pose actions.
 */
class PoseAction{
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


    bool goToTable(int id) {
		switch (id) {
		    case 1:
		        // Final position for BLUE
		        return navigateRobotToGoal(8.15, -2.1, 0.0, -110.0);
		    case 2:
		        // 2nd waypoint
		        navigateRobotToGoal(8.40, -4.2, 0.0, 180.0);
		        // Final position for GREEN
		        return navigateRobotToGoal(7.50, -4.00, 0.0, 70.0);
		    case 3:
		        // Final position for RED
		        return navigateRobotToGoal(7.20, -2.1, 0.0, -50.0);
		    default:
		        ROS_ERROR("Error on selecting object ordering");
		        return false;
		}
    }

    bool goToScanPosition(int id) {
        if (id == 1 || id == 3) {
            feedback_.status = 5;
            as_.publishFeedback(feedback_);
            navigateRobotToGoal(8.4, -2.2, 0.0, 0.0);
        }

        // 2nd Waypoint to look at the cylinders from the center
        feedback_.status = 1;
        as_.publishFeedback(feedback_);
        navigateRobotToGoal(10.3, -4.3, 0.0, 0.0);
        return navigateRobotToGoal(11.33, -2.5, 0.0, 93.0);;
    }
    
     bool goToCylinder(geometry_msgs::Pose pose) {
        feedback_.status = 1;
        as_.publishFeedback(feedback_);
        
        return navigateRobotToGoal(pose);
    }
    
    bool goToHome(){
       ROS_INFO("Operation 0, reaching HOME");
       return  navigateRobotToGoal(8.4, 0.0, 0.0, 0.0);
    }

    /**
     * @brief Callback function for executing the navigation action.
     * @param goal The goal for the pose action.
     */
    void executeCB(const assignment2::PoseGoalConstPtr &goal) {
        
        //Initializing data for goalPosition
        int id = goal->id;

        feedback_.status = 0;
        as_.publishFeedback(feedback_);
        
        geometry_msgs::Pose pose;

        switch (goal->operation) 
        {        
            case 1:
                ROS_INFO("Operation 1, reaching the table passing through home position");
                feedback_.status = 1;
                as_.publishFeedback(feedback_);
                
                executionDone = goToHome();
                
                if (executionDone) {
                    executionDone = goToTable(id);
                }
                
                break;

            case 2:
                ROS_INFO("Operation 2, reaching the scan position");
                executionDone = goToScanPosition(id);
                break;
                
            case 3:
            	ROS_INFO("Operation 3, reaching the place position");

				pose = goal->detection.pose.pose.pose;
            	executionDone = goToCylinder(pose);
            	break;
       
       		case 4:
       		   	ROS_INFO("Operation 4, reaching home position");
       			executionDone = goToHome();
                break;


            default:
                ROS_ERROR("Error while passing navigation operation (table, scan, cylinder, home)");
                break;
        }

        if (executionDone) {
            feedback_.status = 2; // Sending REACHED_GOAL to feedback
            as_.publishFeedback(feedback_);
            result_.arrived = executionDone;
            as_.setSucceeded(result_);
        } else {
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
