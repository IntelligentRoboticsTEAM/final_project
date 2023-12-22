#include "navigation_methods.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

bool navigateRobotToGoal(const Position& goalPosition) 
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //define goal
    ros::Time startTime = ros::Time::now();
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = startTime;
    
    //set goal position
    goal.target_pose.pose.position.x = goalPosition.x;
    goal.target_pose.pose.position.y = goalPosition.y;
    goal.target_pose.pose.position.z = goalPosition.z;
    
    // Set goal orientation
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, goalPosition.yaw);
    myQuaternion.normalize();
    goal.target_pose.pose.orientation = tf2::toMsg(myQuaternion);

	//send goal to ActionServer
    ac.sendGoal(goal);

    // Wait for the robot to reach the goal before a fixed timeout
    bool goalReached = ac.waitForResult(ros::Duration(60.0));
	/*
    if (goalReached) {
        ROS_INFO("Robot reached the goal");
        return true;
    } else {
        ROS_INFO("Robot failed to reach the goal");
        return false;
    }
    */
}


