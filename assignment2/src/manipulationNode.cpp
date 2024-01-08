// ROS headers
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <ros/topic.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Our headers
#include <assignment2/ArmAction.h>
#include "utils.h"

// Std C++ headers
#include <exception>
#include <string>
#include <vector>
#include <map>

const std::string frameName = "base_footprint";

class ArmAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<assignment2::ArmAction> as_;
    std::string action_name_;
    assignment2::ArmFeedback feedback_;
    assignment2::ArmResult result_;

public:

    ArmAction(std::string name) : as_(nh_, name, boost::bind(&ArmAction::executeCB, this, _1), false), action_name_(name)
    {
    	as_.start();
    }
    
    ~ArmAction(void){}

    bool pickObject(geometry_msgs::Pose pose, ros::AsyncSpinner& spinner){

    // Creating PoseStamped  
        geometry_msgs::PoseStamped goalPose;
        goalPose.header.frame_id = frameName;
        goalPose.pose.position.x = pose.position.x;
        goalPose.pose.position.y = pose.position.y;
        goalPose.pose.position.z = pose.position.z;
        goalPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose.orientation.x, pose.orientation.y, pose.orientation.z);        

    // Creating  MoveGroupInterface group_arm_torso
        //select group of joints
        moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
        group_arm_torso.setPlannerId("SBLkConfigDefault");
        group_arm_torso.setPoseReferenceFrame(frameName);
        group_arm_torso.setPoseTarget(goalPose);

    // Planning the movement of the arm
        ROS_INFO_STREAM("Planning to move " <<
                        group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                        group_arm_torso.getPlanningFrame());

        group_arm_torso.setStartStateToCurrentState();
        group_arm_torso.setMaxVelocityScalingFactor(1.0);

    // Creaing plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        //set maximum time to find a plan
        group_arm_torso.setPlanningTime(5.0);
        bool success = bool(group_arm_torso.plan(my_plan));

        if ( !success )
            throw std::runtime_error("No plan found");

        ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
        ros::Time start = ros::Time::now();

    // Execute the Movement
        moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
        if (!bool(e))
            throw std::runtime_error("Error executing plan");
        else
            ROS_INFO("Here I should close the gripper");
            //controlGripper(true); 

        ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

        spinner.stop();

        return true;

        // feedback_.status = 0;
        // as_.publishFeedback(feedback_);

        // feedback_.status = 1;
        // as_.publishFeedback(feedback_);
    }

    bool placeObject(geometry_msgs::Pose goalPose, ros::AsyncSpinner& spinner){


        spinner.stop();
        return true;
    }


    /**
     * @brief Callback function for executing the navigation action.
     * @param goal The goal for the pose action.
     */
    void executeCB(const assignment2::ArmGoalConstPtr &goal) { 

		bool objectPicked = false;
		bool objectPlaced = false;	

        geometry_msgs::Pose goalPose = goal.detections.pose.pose.pose;
		
        switch(goal->request){
            case 1:
                objectPicked = pickObject(goalPose);
                if (objectPicked){
                    result_.objectPicked = objectPicked;
                    as_.setSucceeded(result_);
                } else {
                    result_.objectPicked = objectPicked;
                    as_.setAborted(result_);
                }
                break;
            case 2:
                objectPlaced = placeObject(goalPose);
                if (objectPlaced){
                    result_.objectPlaced = objectPlaced;
                    as_.setSucceeded(result_);
                } else {
                    result_.objectPlaced = objectPlaced;
                    as_.setAborted(result_);
                }
                break;
            default:
                ROS_ERROR("Not a possible choice.");
                break;
        }
    }
};


int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "manipulationNode");
    ArmAction server("manipulationNode");
    ROS_INFO("Manipulation Server is running...");
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::spin(); 


	return 0;
}
