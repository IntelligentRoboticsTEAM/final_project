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
    bool executionDone = false;

    PoseAction(std::string name) : as_(nh_, name, boost::bind(&PoseAction::executeCB, this, _1), false), action_name_(name)
    {
        as_.start();
    }

    ~PoseAction(void){}

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        //code
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

        int numberOfObstacle = 0;
        std::vector<Obstacle> obstacles;

        //feedback_.status = 1;
        executionDone = navigateRobotToGoal(goalPosition);
        
        ir2324_group_10::PoseResult result;

        result.arrived = executionDone;
        as_.setSucceeded(result);
    }

    // void startTimerForScanning() {
    //     ros::Timer timer = nh_.createTimer(ros::Duration(1.0), boost::bind(&PoseAction::scanAfterNavigation, this));
    // }

    // void scanAfterNavigation() {
    //     if (executionDone) {
    //         ros::NodeHandle n;
    //         ros::Subscriber sub = n.subscribe("/scan", 1000, &PoseAction::scanCallback, this);
    //         // Start scanning obstacles or perform any other action after navigation
    //     }
    // }
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
