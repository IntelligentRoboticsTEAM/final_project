#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <project1/PoseAction.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "client_pose");
    actionlib::SimpleActionClient<project1::PoseAction> ac("pose", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); // will wait for infinite time
    ROS_INFO("Action server started.");

    project1::PoseGoal goal;

    ROS_INFO("Enter desired x, y, z, and theta values:");
    std::cin >> goal.x >> goal.y >> goal.z >> goal.theta;

    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            const auto& result = *ac.getResult();
            ROS_INFO("Received result: Arrived - %s", (result.arrived ? "true" : "false"));
        }
    } else {
        ROS_INFO("Action did not finish before the timeout.");
    }

    return 0;
}
