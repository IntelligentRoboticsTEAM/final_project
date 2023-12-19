#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ir2324_group_10/PoseAction.h>
#include <sensor_msgs/LaserScan.h>
#include "navigation_methods.h"

class PoseAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ir2324_group_10::PoseAction> as_;
    std::string action_name_;
    ir2324_group_10::PoseFeedback feedback_;
    ir2324_group_10::PoseResult result_;

public:
    PoseAction(std::string name) : as_(nh_, name, boost::bind(&PoseAction::executeCB, this, _1), false), action_name_(name)
    {
        as_.start();
    }

    ~PoseAction(void){}

    void callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // Process the laser scan data received from the /scan topic
        // Perform obstacle detection or any other required operations
    }

    void executeCB(const ir2324_group_10::PoseGoalConstPtr &goal) {
        
        //ros::Rate(1);
        Position goalPosition;
        goalPosition.x = goal->x;
        goalPosition.y = goal->y;
        goalPosition.z = goal->z;
        goalPosition.yaw = goal->theta_z;

        //feedback_.status = 0;

        ROS_INFO("Received goal: x=%f, y=%f, z=%f, theta_z=%f", goalPosition.x, goalPosition.y, goalPosition.z, goalPosition.yaw);

        bool success_move = false;
        bool success_scan = true;
        int numberOfObstacle = 0;
        std::vector<Obstacle> obstacles;

        //feedback_.status = 1;
        success_move = navigateRobotToGoal(goalPosition);

        /*if (success_move) {
            // Update feedback when the robot reaches the goal
            feedback_.status = 2;  // 2 represents reaching the goal

            // Start scanning obstacles
            feedback_.status = 3;  // 3 represents starting obstacle scanning
            //success_scan = scanObstacles(obstacles, numberOfObstacles);

            if(success_scan)
                feedback_.status = 4;  // 4 represents ending obstacle scanning
        }*/
        

        ir2324_group_10::PoseResult result;

        if (success_move && success_scan)
            result.arrived = true;
        as_.setSucceeded(result);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose");
    PoseAction server("pose");
    ROS_INFO("Server is running...");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/scan", 1000, &PoseAction::callback, &server);
    ros::spin();
    return 0;
}
