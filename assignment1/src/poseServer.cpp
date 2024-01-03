#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <assignment1/PoseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/topic.h>
#include "navigation_methods.h"
#include "scan_methods.h"

/**
 * @brief Defines a class for handling pose actions.
 */
class PoseAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<assignment1::PoseAction> as_;
    std::string action_name_;
    assignment1::PoseFeedback feedback_;
    assignment1::PoseResult result_;

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
    void executeCB(const assignment1::PoseGoalConstPtr &goal) { 

		feedback_.status = 0;
        as_.publishFeedback(feedback_);

		// Initializing data for goalPosition
        Position goalPosition;
        goalPosition.x = goal->x;
        goalPosition.y = goal->y;
        goalPosition.z = goal->z;
        goalPosition.yaw = goal->theta_z;
        ROS_INFO("Received goal: x=%f, y=%f, z=%f, theta_z=%f", goalPosition.x, goalPosition.y, goalPosition.z, goalPosition.yaw);

		// Navigation
        feedback_.status = 1;
        as_.publishFeedback(feedback_);
        executionDone = navigateRobotToGoal(goalPosition);

		if(executionDone)
		{
			//Sending REACHED_GOAL to feedback
		    feedback_.status = 2;
		    as_.publishFeedback(feedback_);
			
			// Once we are in the correct location we read the /scan topic contents
		    ros::NodeHandle n;
			boost::shared_ptr<const sensor_msgs::LaserScan> msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n);
		    
			// Initializing the structure to store the obstacles data
        	int numberOfObstacle = 0;
        	std::vector<Obstacle> obstacles;
			// 1) definitions needed
			const std::vector<float> ranges = msg->ranges;
			const float angle_increment = msg->angle_increment;
			const float angle_min = msg->angle_min;
			float th_distance = 0.8;    // distance between two adjacent pts		

			// Scanning
		    feedback_.status = 3;
			as_.publishFeedback(feedback_);

			// 2) scan the obstacles
			std::vector<std::vector<float>> rangeClusters = clusterRanges(ranges, th_distance);
			obstacles = findObstacles1(rangeClusters, angle_min, angle_increment);
			
			// 2.1) convert the obstacles' vector into a message
			std::vector<assignment1::Obstacle> msgObstacles = convertToMsgType(obstacles);
			
			// Scan ended
			feedback_.status = 4;
			as_.publishFeedback(feedback_);
			result_.obstacles = msgObstacles;
		    as_.setSucceeded(result_);

		}else {
		ROS_INFO("Navigation aborted - Timeout reached");
		as_.setAborted(result_);}
    }


/**
     * @brief Convert std::vector<Obstacle> to assignment1::Obstacle[] (--> message type)
     * @param obstacles The obstacles to convert.
     * @return Vector of obstacles in message format.
     */
    std::vector<assignment1::Obstacle> convertToMsgType(const std::vector<Obstacle>& obstacles) {
        std::vector<assignment1::Obstacle> msgObstacles;

        for (const Obstacle& obstacle : obstacles) 
        {
            assignment1::Obstacle msgObstacle;
            msgObstacle.x = obstacle.getX();
            msgObstacle.y = obstacle.getY();
            msgObstacle.radius = obstacle.getRadius();

            msgObstacles.push_back(msgObstacle);
        }

        return msgObstacles;
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "pose");
    PoseAction server("pose");
    ROS_INFO("Server is running...");
    
    ros::spin();

    return 0;
}
