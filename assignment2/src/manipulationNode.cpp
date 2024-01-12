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
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Our headers
#include <assignment2/ArmAction.h>
#include "utils.h"

// Std C++ headers
#include <exception>
#include <string>
#include <vector>
#include <map>

const std::string frameName = "base_footprint";
const double tau = 2 * M_PI;

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
    
    void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<apriltag_ros::AprilTagDetection> detections)
	{
	  std::vector<moveit_msgs::CollisionObject> collision_objects;
	  ROS_INFO("Inside addCollisionObjects");

	  moveit_msgs::CollisionObject table_object;
	  table_object.id = "table";
	  table_object.header.frame_id = "odom";
	  
	  // Define the shape of the collision object
	  shape_msgs::SolidPrimitive primitive;
	  primitive.type = shape_msgs::SolidPrimitive::BOX;
	  primitive.dimensions.resize(3);
	  primitive.dimensions[0] = 1.0;  // x dimension
	  primitive.dimensions[1] = 1.0;  // y dimension
	  primitive.dimensions[2] = 0.75;  // z dimension

	  // Set the pose of the collision object in the base frame
	  geometry_msgs::Pose table_pose;
	  table_pose.orientation.x = 0.0;
	  table_pose.orientation.y = 0.0;
	  table_pose.orientation.z = 1.0;
	  table_pose.orientation.w = 0.0;  // Quaternion identity
	  table_pose.position.x = 7.70;
	  table_pose.position.y = -3.00;
	  table_pose.position.z = 0.75;
	  
	  table_object.operation = 0; //ADD

	  // Add the primitive to the collision object
	  table_object.primitives.push_back(primitive);
	  table_object.primitive_poses.push_back(table_pose);
	
	  collision_objects.push_back(table_object);

	  for(int i = 0; i < detections.size(); i++)
	  {
		ROS_INFO("Loop number: %d", (int)i);

		moveit_msgs::CollisionObject obstacle_object;
		shape_msgs::SolidPrimitive primitive;
		geometry_msgs::Pose object_pose;
		
		
		ROS_INFO("Id at iter: %d, is number: %d", (int)i, (int)detections[i].id[0]);
		switch((int)detections[i].id[0]){
			case 1:
				ROS_INFO("Switch 1");
				obstacle_object.id = std::to_string(detections[i].id[0]);
				obstacle_object.header.frame_id = "base_footprint";
				
				primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
	  			primitive.dimensions.resize(3);
	 			primitive.dimensions[0] = detections[i].size[0] + 0.02;  // x dimension
	  			primitive.dimensions[1] = detections[i].size[0] + 0.02;  // y dimension
	  			primitive.dimensions[2] = detections[i].pose.pose.pose.position.z - 0.75;  // z dimension
				
				object_pose = detections[i].pose.pose.pose;
				
				obstacle_object.operation = 0; //ADD
				
				obstacle_object.primitives.push_back(primitive);
	  			obstacle_object.primitive_poses.push_back(object_pose);
				
				collision_objects.push_back(obstacle_object);
			    ROS_INFO("fine case 1");
				break;
			case 2: //da rivedere perche ha una forma strana
				ROS_INFO("Switch 2");
				obstacle_object.id = std::to_string(detections[i].id[0]);
				obstacle_object.header.frame_id = "base_footprint";
				
				primitive.type = shape_msgs::SolidPrimitive::BOX;
	  			primitive.dimensions.resize(3);
	 			primitive.dimensions[0] = detections[i].size[0] + 0.02;  // x dimension
	  			primitive.dimensions[1] = detections[i].size[0] + 0.02;  // y dimension
	  			primitive.dimensions[2] = detections[i].pose.pose.pose.position.z - 0.75;  // z dimension
				
				object_pose = detections[i].pose.pose.pose;
				
				obstacle_object.operation = 0; //ADD
				
				obstacle_object.primitives.push_back(primitive);
	  			obstacle_object.primitive_poses.push_back(object_pose);
				
				collision_objects.push_back(obstacle_object);
				ROS_INFO("fine case 2");
				break;
			case 3:
				ROS_INFO("Switch 1");
				obstacle_object.id = std::to_string(detections[i].id[0]);
				obstacle_object.header.frame_id = "base_footprint";
				
				primitive.type = shape_msgs::SolidPrimitive::BOX;
	  			primitive.dimensions.resize(3);
	 			primitive.dimensions[0] = detections[i].size[0] + 0.02;  // x dimension
	  			primitive.dimensions[1] = detections[i].size[0] + 0.02;  // y dimension
	  			primitive.dimensions[2] = detections[i].pose.pose.pose.position.z - 0.75;  // z dimension
				
				object_pose = detections[i].pose.pose.pose;
				
				obstacle_object.operation = 0; //ADD
				
				obstacle_object.primitives.push_back(primitive);
	  			obstacle_object.primitive_poses.push_back(object_pose);
				
				collision_objects.push_back(obstacle_object);
				ROS_INFO("fine case 3");
				break;
			default:
				ROS_INFO("Switch default");
				obstacle_object.id = std::to_string(detections[i].id[0]);
				obstacle_object.header.frame_id = "base_footprint";
				
				primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
	  			primitive.dimensions.resize(3);
	 			primitive.dimensions[0] = detections[i].size[0] + 0.02;  // x dimension
	  			primitive.dimensions[1] = detections[i].size[0] + 0.02;  // y dimension
	  			primitive.dimensions[2] = detections[i].pose.pose.pose.position.z - 0.75;  // z dimension
				
				object_pose = detections[i].pose.pose.pose;
				
				obstacle_object.operation = 0; //ADD
				
				obstacle_object.primitives.push_back(primitive);
	  			obstacle_object.primitive_poses.push_back(object_pose);
				
				collision_objects.push_back(obstacle_object);
				ROS_INFO("fine default");
				break;
		}
		
	  }

	  for (int i = 0; i < collision_objects.size(); i++)
		ROS_INFO("Collision object number: %s", collision_objects[i].id.c_str());

	  ROS_INFO("Collision objects created, number of objects in collision_objects vector: %d", (int)collision_objects.size());
	  planning_scene_interface.applyCollisionObjects(collision_objects);
	}

	
    void openGripper(trajectory_msgs::JointTrajectory& posture)
	{
	  ROS_INFO("Opening gripper");
	  // BEGIN_SUB_TUTORIAL open_gripper
	  /* Add both finger joints of panda robot. */
	  posture.joint_names.resize(2);
	  posture.joint_names[0] = "gripper_left_finger_link";
	  posture.joint_names[1] = "gripper_right_finger_link";

	  /* Set them as open, wide enough for the object to fit. */
	  posture.points.resize(1);
	  posture.points[0].positions.resize(2);
	  posture.points[0].positions[0] = 0.04;
	  posture.points[0].positions[1] = 0.04;
	  posture.points[0].time_from_start = ros::Duration(0.5);
	  // END_SUB_TUTORIAL
	}

	void closedGripper(trajectory_msgs::JointTrajectory& posture)
	{
	  ROS_INFO("Closing gripper");
	  
	  posture.joint_names.resize(2);
	  posture.joint_names[0] = "gripper_left_finger_link";
	  posture.joint_names[1] = "gripper_right_finger_link";

	  /* Set them as closed. */
	  posture.points.resize(1);
	  posture.points[0].positions.resize(2);
	  posture.points[0].positions[0] = 0.00;
	  posture.points[0].positions[1] = 0.00;
	  posture.points[0].time_from_start = ros::Duration(0.5);

	}
	
	bool pickObject(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<apriltag_ros::AprilTagDetection> detections, int requestedID){

		  int detection_index = 0;
		  while(detections[detection_index].id[0] != requestedID){
		  	detection_index++;
		  } 
		  
		  ROS_INFO("Starting the picking procedure");
		  std::vector<moveit_msgs::Grasp> grasps;
		  grasps.resize(1);

		  // Setting grasp pose
		  // ++++++++++++++++++++++
		  // This is the pose of panda_link8. |br|
		  // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
		  // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
		  // transform from `"panda_link8"` to the palm of the end effector.
		  grasps[0].grasp_pose.header.frame_id = "base_footprint";
		  grasps[0].grasp_pose.pose.orientation = detections[detection_index].pose.pose.pose.orientation;
		  grasps[0].grasp_pose.pose.position = detections[detection_index].pose.pose.pose.position;

		  // Setting pre-grasp approach
		  // ++++++++++++++++++++++++++
		  /* Defined with respect to frame_id */
		  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_footprint";
		  /* Direction is set as positive x axis */
		  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
		  grasps[0].pre_grasp_approach.min_distance = 0.095;
		  grasps[0].pre_grasp_approach.desired_distance = 0.115;

		  // Setting post-grasp retreat
		  // ++++++++++++++++++++++++++
		  /* Defined with respect to frame_id */
		  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_footprint";
		  /* Direction is set as positive z axis */
		  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
		  grasps[0].post_grasp_retreat.min_distance = 0.1;
		  grasps[0].post_grasp_retreat.desired_distance = 0.25;

		  // Setting posture of eef before grasp
		  // +++++++++++++++++++++++++++++++++++
		  openGripper(grasps[0].pre_grasp_posture);
		  // END_SUB_TUTORIAL

		  // BEGIN_SUB_TUTORIAL pick2
		  // Setting posture of eef during grasp
		  // +++++++++++++++++++++++++++++++++++
		  closedGripper(grasps[0].grasp_posture);
		  // END_SUB_TUTORIAL

		  // BEGIN_SUB_TUTORIAL pick3
		  // Set support surface as table1.
		  move_group.setSupportSurfaceName("table");
		  // Call pick to pick up the object using the grasps given
		  move_group.pick(std::to_string(detections[detection_index].id[0]), grasps);
		  // END_SUB_TUTORIAL
		  return true;
	}
	
    bool pickTutorial(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<apriltag_ros::AprilTagDetection> detections){

    	// Creating PoseStamped  
        geometry_msgs::PoseStamped goalPose;
        goalPose.header.frame_id = "base_footprint";
        goalPose.pose = detections[0].pose.pose.pose;        

    	// Creating  MoveGroupInterface group_arm_torso
        //select group of joints
        move_group.setPlannerId("SBLkConfigDefault");
        move_group.setPoseReferenceFrame(frameName);
        move_group.setPoseTarget(goalPose);

    	// Planning the movement of the arm
        ROS_INFO_STREAM("Planning to move " <<
                        move_group.getEndEffectorLink() << " to a target pose expressed in " <<
                        move_group.getPlanningFrame());

        move_group.setStartStateToCurrentState();
        move_group.setMaxVelocityScalingFactor(1.0);

    	// Creaing plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        //set maximum time to find a plan
        move_group.setPlanningTime(5.0);
        bool success = bool(move_group.plan(my_plan));

        if ( !success )
            throw std::runtime_error("No plan found");

        ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
        ros::Time start = ros::Time::now();

    	// Execute the Movement
        moveit::core::MoveItErrorCode e = move_group.move();
        if (!bool(e))
            throw std::runtime_error("Error executing plan");
        else
            ROS_INFO("Here I should close the gripper");
            //controlGripper(true); 

        ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

        //spinner.stop();

        return true;

        // feedback_.status = 0;
        // as_.publishFeedback(feedback_);

        // feedback_.status = 1;
        // as_.publishFeedback(feedback_);
    }

    bool placeObject(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<apriltag_ros::AprilTagDetection> detections){


        //spinner.stop();
        return true;
    }


    /**
     * @brief Callback function for executing the navigation action.
     * @param goal The goal for the pose action.
     */
    void executeCB(const assignment2::ArmGoalConstPtr &goal) { 
		
		ROS_INFO("Inside executeCB");
		
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		ROS_INFO("Inside executeCB, after planning");
		
  		moveit::planning_interface::MoveGroupInterface group("arm_torso");
  		ROS_INFO("Inside executeCB, after group");
  		group.setPlanningTime(45.0);

	  	addCollisionObjects(planning_scene_interface, goal->detections);
	  	ROS_INFO("Inside executeCB, adter collision objets adding");
	  	
		bool objectPicked = false;
		bool objectPlaced = false;	

        std::vector<apriltag_ros::AprilTagDetection> detections = goal->detections;
        geometry_msgs::PoseWithCovarianceStamped p1 = detections[0].pose;
        geometry_msgs::PoseWithCovariance p2 = p1.pose;
        geometry_msgs::Pose goalPose = p2.pose;
		
        switch(goal->request){
            case 1:
                objectPicked = pickObject(group, detections, goal->id);
                if (objectPicked){
                    result_.objectPicked = objectPicked;
                    as_.setSucceeded(result_);
                } else {
                    result_.objectPicked = objectPicked;
                    as_.setAborted(result_);
                }
                break;
            case 2:
                objectPlaced = placeObject(group, detections);
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
    
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    
    ros::spin(); 


	return 0;
}
