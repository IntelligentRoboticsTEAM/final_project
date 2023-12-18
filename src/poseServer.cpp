#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <project1/PoseAction.h>

class PoseAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<project1::PoseAction> as_;
    std::string action_name_;
    project1::PoseFeedback feedback_;
    project1::PoseResult result_;

public:
    PoseAction(std::string name) : as_(nh_, name, boost::bind(&PoseAction::executeCB, this, _1), false), action_name_(name)
    {
        as_.start();
    }

    ~PoseAction(void){}

    void executeCB(const project1::PoseGoalConstPtr &goal) {
        
        ros::Rate(1);

        ROS_INFO("Received goal: x=%f, y=%f, z=%f, theta=%f", goal->x, goal->y, goal->z, goal->theta);
        
        bool success = true;

        project1::PoseResult result;
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
