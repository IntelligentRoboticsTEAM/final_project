#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <assignment2/PoseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/topic.h>
#include "headers/navigation_methods.h"
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


    bool goToTable(int id) 
    {
		feedback_.status = 1;
		as_.publishFeedback(feedback_);
		
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
		
		feedback_.status = 2;
		as_.publishFeedback(feedback_);
		
	            
        feedback_.status = 0;
		as_.publishFeedback(feedback_);
		
    }

    bool goToScanPosition(int id) {
        if (id == 1 || id == 3) 
        {

			feedback_.status = 1;
			as_.publishFeedback(feedback_);
            
            navigateRobotToGoal(8.4, -2, 0.0, 0.0);
            
            feedback_.status = 2;
            as_.publishFeedback(feedback_);
            
            feedback_.status = 0;
			as_.publishFeedback(feedback_);
		
            
        }else
        {
        	feedback_.status = 1;
			as_.publishFeedback(feedback_);
            
            // tiago started crashing into the table after taking the 1st video, so we added this waypoint
            navigateRobotToGoal(8.0, -4.3, 0.0, 0.0);
            
            feedback_.status = 2;
            as_.publishFeedback(feedback_);
            
            feedback_.status = 0;
			as_.publishFeedback(feedback_);
        }
        

        // 2nd Waypoint to look at the cylinders from the center
		feedback_.status = 1;
		as_.publishFeedback(feedback_);
		
        navigateRobotToGoal(10.3, -4.3, 0.0, 0.0);
        
        bool returned = navigateRobotToGoal(11.33, -2.5, 0.0, 93.0);
        if(returned){
        	feedback_.status = 2;
            as_.publishFeedback(feedback_);
        }
        
        feedback_.status = 0;
		as_.publishFeedback(feedback_);
		
        
        return returned;
    }
    
     bool goToCylinder(geometry_msgs::Pose pose) {

		feedback_.status = 1;
		as_.publishFeedback(feedback_);
        
        return navigateRobotToGoal(pose.position.x, pose.position.y, pose.position.z, 90.0);
    }
    
    bool goToHome(int id){
       ROS_INFO("Operation 0, reaching HOME");
       
       navigateRobotToGoal(10.3, -4.3, 0.0, 180.0);
       
       bool returned = navigateRobotToGoal(8.4, 0.0, 0.0, 0.0);	 // HOME
       if(returned){
        	feedback_.status = 2;
            as_.publishFeedback(feedback_);
        }
        
        feedback_.status = 0;
		as_.publishFeedback(feedback_);
		
        
        return returned;
    }

    /**
     * @brief Callback function for executing the navigation action.
     * @param goal The goal for the pose action.
     */
    void executeCB(const assignment2::PoseGoalConstPtr &goal) 
    {
    // STOPPED
        feedback_.status = 0;
        as_.publishFeedback(feedback_);
        
        //Initializing data for goalPosition
        int id = goal->id;
        geometry_msgs::Pose pose;
        
	// MOVING
     //   feedback_.status = 1;
     //   as_.publishFeedback(feedback_);
        
        switch (goal->operation) 
        {        
            case 1:
                ROS_INFO("Operation 1, reaching the table passing through home position");
                feedback_.status = 1;
                as_.publishFeedback(feedback_);
                
                executionDone = navigateRobotToGoal(8.4, 0.0, 0.0, 0.0);
                if (executionDone) 
                	executionDone = goToTable(id);
                             
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
       		   	ROS_INFO("Operation 4, reaching home position, then table position");
       			executionDone = goToHome(id);
       			if (executionDone) 
       				executionDone = goToTable(id);
       				
                break;
                
            case 5:
               	ROS_INFO("Operation 5, reaching HOME position.");
               	executionDone = goToHome(id);
               	
               	break;
               	
            default:
                ROS_ERROR("Error in Navigation CallBack.");
                break;
        }

	// ARRIVED
        if (executionDone) {
        //    feedback_.status = 2; // Sending REACHED_GOAL to feedback
        //    as_.publishFeedback(feedback_);
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
