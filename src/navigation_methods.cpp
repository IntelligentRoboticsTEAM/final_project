#include "navigation_methods.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

bool navigateRobotToGoal(const Position& goalPosition) 
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ros::Time startTime = ros::Time::now();

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = startTime;
    goal.target_pose.pose.position.x = goalPosition.x;
    goal.target_pose.pose.position.y = goalPosition.y;
    goal.target_pose.pose.position.z = goalPosition.z;
    
    // Set orientation if required
    //goal.target_pose.pose.orientation.z = goalPosition.yaw;
    
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, goalPosition.yaw);
    myQuaternion.normalize();
    
    goal.target_pose.pose.orientation = tf2::toMsg(myQuaternion);

    ac.sendGoal(goal);

    // Wait for the robot to reach the goal or a timeout
    bool goalReached = ac.waitForResult(ros::Duration(60.0));

    ros::Time endTime = ros::Time::now();
    ros::Duration duration = endTime - startTime;

    if (goalReached) {
        ROS_INFO("Robot reached the goal");
        ROS_INFO("Duration: %f seconds", duration.toSec());
        return true;
    } else {
        ROS_INFO("Robot failed to reach the goal");
        ROS_INFO("Duration: %f seconds", duration.toSec());
        return false;
    }
}

