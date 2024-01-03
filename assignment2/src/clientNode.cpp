#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>

#include "assignment1/PoseAction.h" 
#include "tiago_iaslab_simulation/Objs.h" 


typedef actionlib::SimpleActionClient<your_package::YourActionServerAction> ActionClient;

bool handleService(tiago_iaslab_simulation::Objs::Request &req, tiago_iaslab_simulation::Objs::Response &res) {
    std::vector<int32_t> receivedArray = req.your_array;

    ROS_INFO("Received array from human_node:");
    for (int i = 0; i < receivedArray.size(); ++i) {
        ROS_INFO("Value %d: %d", i, receivedArray[i]);
    }

    ActionClient ac("poseRevisited", true); 
    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer();

    ROS_INFO("Action server started.");

    // Create and send a goal to the action server
    assignment1::PoseGoal goal;
    goal.x = 8;
    goal.y = -2.2;
    goal.z = 0.00;
    goal.theta_z = -90;
    ac.sendGoal(goal);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        res.success = true;

    } else {
        ROS_INFO("Action did not finish before the timeout.");
        res.success = false;
    }

    return res.success;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "service_server_node");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("human_objects_srv", handleService); 

    ROS_INFO("Service server node ready to receive arrays from human_node.");

    ros::spin();
    return 0;
}
