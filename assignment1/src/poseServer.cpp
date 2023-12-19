#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <assignment1/PoseAction.h>
#include "navigation_methods.h"

class PoseAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<assignment1::PoseAction> as_;
    std::string action_name_;
    assignment1::PoseFeedback feedback_;
    assignment1::PoseResult result_;

public:
    PoseAction(std::string name) : as_(nh_, name, boost::bind(&PoseAction::executeCB, this, _1), false), action_name_(name)
    {
        as_.start();
    }

    ~PoseAction(void){}

    void executeCB(const assignment1::PoseGoalConstPtr &goal) {
        
        ros::Rate(1);

        Position goalPosition;
        goalPosition.x = goal->x;
        goalPosition.y = goal->y;
        goalPosition.z = goal->z;

        ROS_INFO("Received goal: x=%f, y=%f, z=%f", goalPosition.x, goalPosition.y, goalPosition.z);

        bool success = false;
        int numberOfObstacles = 0;
        std::vector<Obstacle> obstacles;

        success = navigateRobotToGoal(goalPosition);
        
        //numberOfObstacles = scanObstacles(obstacles);

        assignment1::PoseResult result;
        result.arrived = success;
        as_.setSucceeded(result);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose");
    PoseAction server("pose");
    ROS_INFO("Server is running...");
    ros::spin();
    return 0;
}
