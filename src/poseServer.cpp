#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ir2324_group_10/PoseAction.h>
#include <sensor_msgs/LaserScan.h>
#include "navigation_methods.h"
#include "scan_methods.h"

class PoseAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ir2324_group_10::PoseAction> as_;
    std::string action_name_;
    ir2324_group_10::PoseFeedback feedback_;
    ir2324_group_10::PoseResult result_;
    ros::Publisher feedback_pub_; 

public:
    bool executionDone = false;

    PoseAction(std::string name) : as_(nh_, name, boost::bind(&PoseAction::executeCB, this, _1), false), action_name_(name)
    {
    	as_.start();
    }
    
    ~PoseAction(void){}

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

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
        
        // 3) publish clusters cartesian coordinates
        ROS_INFO("Values in the array:");

        int i = 1;
        for (Obstacle o : obstacles) 
        {    
            ROS_INFO("--------------------");
            ROS_INFO("Object n.%d:\nHas coordinates (X = %f, Y = %f)\nHas size (radius = %f)", 
                    i, o.getX(), o.getY(), o.getRadius());
            i++;
        }
        
        feedback_.status = 4;
        as_.publishFeedback(feedback_);

        ir2324_group_10::PoseResult result;
        result.arrived = true;
        result.obstacles = msgObstacles; 
        as_.setSucceeded(result);
    }

    void executeCB(const ir2324_group_10::PoseGoalConstPtr &goal) { 
        //ros::Rate(1);
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

        scanAfterNavigation();
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



    void scanAfterNavigation() {
        feedback_.status = 3;
        as_.publishFeedback(feedback_);

        if (executionDone) {
            ros::NodeHandle n;
            ros::Subscriber sub = n.subscribe("/scan", 1000, &PoseAction::scanCallback, this);
        }
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "pose");
    PoseAction server("pose");
    ROS_INFO("Server is running...");

    //server.startTimerForScanning();

    ros::spin();

    return 0;

    // if (server.executionDone) {
    //     // Update feedback when the robot reaches the goal
    //     //feedback_.status = 2;  // 2 represents reaching the goal

    //     // Start scanning obstacles
    //     //feedback_.status = 3;  // 3 represents starting obstacle scanning
    //     ros::NodeHandle n;
    //     ros::Subscriber sub = n.subscribe("/scan", 1000, &PoseAction::scanCallback, &server);
    //     //success_scan = scanObstacles(obstacles, numberOfObstacles);

    //     // if(success_scan)
    //     //     feedback_.status = 4;  // 4 represents ending obstacle scanning
    // }

}
