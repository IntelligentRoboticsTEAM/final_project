#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ir2324_group_10/PoseAction.h>
#include "utils.h"

void feedbackCallback(const ir2324_group_10::PoseFeedbackConstPtr& feedback) {
    int status = feedback->status;

    switch (status) {
        case 0:
            ROS_INFO("Received status: STOPPED");
            break;
        case 1:
            ROS_INFO("Received status: MOVING");
            break;
        case 2:
            ROS_INFO("Received status: REACHED_GOAL");
            break;
        case 3:
            ROS_INFO("Received status: STARTED_SCAN");
            break;
        case 4:
            ROS_INFO("Received status: ENDED_SCAN");
            break;
        default:
            ROS_INFO("Received unknown status");
            break;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "client_pose");
    actionlib::SimpleActionClient<ir2324_group_10::PoseAction> ac("pose", true);
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); // will wait for infinite time
    ROS_INFO("Action server started.");

    ir2324_group_10::PoseGoal goal;
    double degree_theta_z = 0.00;

    ROS_INFO("Enter desired x, y, z, and yaw angle values:\n");
    ROS_INFO("X: ");
    std::cin >> goal.x;     // BEST VALUE TO PICK: goal.x = 11.00;
    ROS_INFO("Y: ");
    std::cin >> goal.y;     // BEST VALUE TO PICK: goal.y = 1.00;
    ROS_INFO("Z: ");
    std::cin >> goal.z;     // BEST VALUE TO PICK: goal.z = 0.00;
    ROS_INFO("Yaw angle: ");
    std::cin >> degree_theta_z; // BEST VALUE TO PICK: goal.theta_z = -90)
    
    goal.theta_z = degreesToRadians(degree_theta_z);
    
    ac.sendGoal(goal, NULL, NULL, &feedbackCallback);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

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


