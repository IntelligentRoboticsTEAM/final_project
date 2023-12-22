#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ir2324_group_10/PoseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/topic.h>
#include "navigation_methods.h"
#include "scan_methods.h"

class PoseAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ir2324_group_10::PoseAction> as_;
    std::string action_name_;
    ir2324_group_10::PoseFeedback feedback_;
    ir2324_group_10::PoseResult result_;

public:
    bool executionDone = false;


    PoseAction(std::string name) : as_(nh_, name, boost::bind(&PoseAction::executeCB, this, _1), false), action_name_(name)
    {
    	as_.start();
    }
    
    ~PoseAction(void){}
	/*
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
		ROS_INFO("scancallback");

		feedback_.status = 3;
	    as_.publishFeedback(feedback_);
	    
	    // 1) definitions
	    const std::vector<float> ranges = msg->ranges;
	    const float angle_increment = msg->angle_increment;
	    const float angle_min = msg->angle_min;
	    float th = 0.2;    // distance between two adjacent pts
	    std::vector<Obstacle> obstacles;

	    // 2) scan the obstacles
	    std::vector<std::vector<float>> rangeClusters = clusterRanges(ranges, th);
	    obstacles = computeAvg(rangeClusters, angle_min, angle_increment);
	    
	    // 2.1) convert the obstacles' vector into a message
	    std::vector<ir2324_group_10::Obstacle> msgObstacles = convertToMsgType(obstacles);
	    
	    feedback_.status = 4;
	    as_.publishFeedback(feedback_);
	    result_.arrived = true;
	    result_.obstacles = msgObstacles;
		 
    }
    */

    void executeCB(const ir2324_group_10::PoseGoalConstPtr &goal) { 

        Position goalPosition;
        goalPosition.x = goal->x;
        goalPosition.y = goal->y;
        goalPosition.z = goal->z;
        goalPosition.yaw = goal->theta_z;

        feedback_.status = 0;
        as_.publishFeedback(feedback_);
        ROS_INFO("Received goal: x=%f, y=%f, z=%f, theta_z=%f", goalPosition.x, goalPosition.y, goalPosition.z, goalPosition.yaw);

        int numberOfObstacle = 0;
        std::vector<Obstacle> obstacles;

        feedback_.status = 1;
        as_.publishFeedback(feedback_);
        executionDone = navigateRobotToGoal(goalPosition);
        feedback_.status = 2;
        as_.publishFeedback(feedback_);
        
        ros::NodeHandle n;
		boost::shared_ptr<const sensor_msgs::LaserScan> msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n);
        
        feedback_.status = 3;
	    as_.publishFeedback(feedback_);
	    
	    // 1) definitions
	    const std::vector<float> ranges = msg->ranges;
	    const float angle_increment = msg->angle_increment;
	    const float angle_min = msg->angle_min;
	    float th = 0.2;    // distance between two adjacent pts

	    // 2) scan the obstacles
	    std::vector<std::vector<float>> rangeClusters = clusterRanges(ranges, th);
	    obstacles = computeAvg(rangeClusters, angle_min, angle_increment);
	    
	    // 2.1) convert the obstacles' vector into a message
	    std::vector<ir2324_group_10::Obstacle> msgObstacles = convertToMsgType(obstacles);
	    
	    feedback_.status = 4;
	    as_.publishFeedback(feedback_);
	    result_.arrived = true;
	    result_.obstacles = msgObstacles;
        
        as_.setSucceeded(result_);
    }

    // Convert std::vector<Obstacle> to ir2324_group_10::Obstacle[]
    std::vector<ir2324_group_10::Obstacle> convertToMsgType(const std::vector<Obstacle>& obstacles) {
        std::vector<ir2324_group_10::Obstacle> msgObstacles;

        for (const Obstacle& obstacle : obstacles) 
        {
            ir2324_group_10::Obstacle msgObstacle;
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
