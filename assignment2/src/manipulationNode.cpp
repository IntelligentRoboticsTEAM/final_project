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
#include <tf2/LinearMath/Quaternion.h>


// Our headers
#include <assignment2/ArmAction.h>
#include "headers/utils.h"

// Std C++ headers
#include <exception>
#include <string>
#include <vector>
#include <map>
#include <cmath>


class ArmAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<assignment2::ArmAction> as_;
    std::string action_name_;
    assignment2::ArmFeedback feedback_;
    assignment2::ArmResult result_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface arm_group;
	moveit::planning_interface::MoveGroupInterface gripper_group;

public:

    ArmAction(std::string name) : as_(nh_, name, boost::bind(&ArmAction::executeCB, this, _1), false), action_name_(name),
    planning_scene_interface(),
    arm_group("arm_torso"),
    gripper_group("gripper")
    {
    	as_.start();
    }
    
    ~ArmAction(void){}
    
     moveit_msgs::CollisionObject addCollisionObjects(std::vector<apriltag_ros::AprilTagDetection> detections)
	{
		std::vector<moveit_msgs::CollisionObject> collision_objects;

		// ADDING TABLE COLLISION OBJECT
		moveit_msgs::CollisionObject table_object;
		table_object.id = "table";
		table_object.header.frame_id = "odom";
		
		// Define the shape of the collision object
		shape_msgs::SolidPrimitive table_primitive;
		table_primitive.type = shape_msgs::SolidPrimitive::BOX;
		table_primitive.dimensions.resize(3);
		table_primitive.dimensions[0] = 0.96;  // x dimension
		table_primitive.dimensions[1] = 0.96;  // y dimension
		table_primitive.dimensions[2] = 0.755;  // z dimension

		// Set the pose of the collision object (odom frame)
		geometry_msgs::Pose table_pose;
		table_pose.orientation.x = 0.0;
		table_pose.orientation.y = 0.0;
		table_pose.orientation.z = 1.0;
		table_pose.orientation.w = 0.0;
		table_pose.position.x = 7.82;
		table_pose.position.y = -2.98;
		table_pose.position.z = 0.375;
		
		// Add the primitive to the collision object
		table_object.primitives.push_back(table_primitive);
		table_object.primitive_poses.push_back(table_pose); //map
		table_object.operation = table_object.ADD;
		
		collision_objects.push_back(table_object);
		
		moveit_msgs::CollisionObject return_object;
		for(int i = 0; i < detections.size(); i++)
		{
			moveit_msgs::CollisionObject obstacle_object;
			shape_msgs::SolidPrimitive obj_primitive;
			geometry_msgs::Pose object_pose;

			obstacle_object.id = std::to_string(detections[i].id[0]); // 1, 2, ... ,  7
			obstacle_object.header.frame_id = "odom";

			switch((int)detections[i].id[0]){
				case 1:					
					obj_primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
					obj_primitive.dimensions.resize(2);
					obj_primitive.dimensions[0] = detections[i].pose.pose.pose.position.z - 0.755;  // height
					obj_primitive.dimensions[1] = sqrt(2*(detections[i].size[0]*detections[i].size[0]))/2 + 0.005;  // radius

					object_pose.position.x = detections[i].pose.pose.pose.position.x;
					object_pose.position.y = detections[i].pose.pose.pose.position.y;
					object_pose.position.z = detections[i].pose.pose.pose.position.z - obj_primitive.dimensions[0] / 2;
					object_pose.orientation = detections[i].pose.pose.pose.orientation;
			
					return_object = obstacle_object;
					
					break;
				case 2: //da rivedere perche ha una forma strana					
					obj_primitive.type = shape_msgs::SolidPrimitive::BOX;
					obj_primitive.dimensions.resize(3);
					obj_primitive.dimensions[0] = detections[i].size[0] + 0.015;  // x dimension
					obj_primitive.dimensions[1] = detections[i].size[0] + 0.015;  // y dimension
					obj_primitive.dimensions[2] = detections[i].size[0] + 0.015;  // z dimension
					
					object_pose.position.x = detections[i].pose.pose.pose.position.x + 0.015;
					object_pose.position.y = detections[i].pose.pose.pose.position.y + 0.015;
					object_pose.position.z = 0.755;
					object_pose.orientation = detections[i].pose.pose.pose.orientation;
					
					return_object = obstacle_object;
					
					break;
					
				case 3:					
					obj_primitive.type = shape_msgs::SolidPrimitive::BOX;
					obj_primitive.dimensions.resize(3);
					obj_primitive.dimensions[0] = detections[i].size[0] + 0.015;  // x dimension
					obj_primitive.dimensions[1] = detections[i].size[0] + 0.015;  // y dimension
					obj_primitive.dimensions[2] = detections[i].size[0] + 0.015;  // z dimension
					
					object_pose.position.x = detections[i].pose.pose.pose.position.x;
					object_pose.position.y = detections[i].pose.pose.pose.position.y;
					object_pose.position.z = detections[i].pose.pose.pose.position.z - obj_primitive.dimensions[2] / 2;
					object_pose.orientation = detections[i].pose.pose.pose.orientation;
					
					return_object = obstacle_object;
					
					break;
					
				default:					
					obj_primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
					obj_primitive.dimensions.resize(2);
					obj_primitive.dimensions[0] = detections[i].pose.pose.pose.position.z - 0.755 + 0.1;  // height
					obj_primitive.dimensions[1] = sqrt(2*(detections[i].size[0]*detections[i].size[0]))/2 + 0.02;  // radius
					
					object_pose.position.x = detections[i].pose.pose.pose.position.x;
					object_pose.position.y = detections[i].pose.pose.pose.position.y;
					object_pose.position.z = detections[i].pose.pose.pose.position.z - 0.1;// - primitive.dimensions[0] / 2;
					object_pose.orientation = detections[i].pose.pose.pose.orientation;
					
					break;
			}

			obstacle_object.operation = obstacle_object.ADD;
			
			obstacle_object.primitives.push_back(obj_primitive);
			obstacle_object.primitive_poses.push_back(object_pose);
			
			collision_objects.push_back(obstacle_object);
		}
		
		planning_scene_interface.applyCollisionObjects(collision_objects);
		
		return return_object;
	}

	
    bool pick(std::vector<apriltag_ros::AprilTagDetection> detections, int requestedID){
    	
		int detection_index = 0;
		  while(detections[detection_index].id[0] != requestedID){
		  	detection_index++;
		}
		
    	// Creating PoseStamped approach/depart pose 
        geometry_msgs::PoseStamped appro_pose;
        appro_pose.header.frame_id = "odom";
        tf2::Quaternion original_quaternion(detections[detection_index].pose.pose.pose.orientation.x, detections[detection_index].pose.pose.pose.orientation.y, detections[detection_index].pose.pose.pose.orientation.z, detections[detection_index].pose.pose.pose.orientation.w);
        tf2::Matrix3x3 m(original_quaternion);
		double roll, pitch, yaw;
		tf2::Quaternion rotation_about_y;
		/*
		geometry_msgs::PoseStamped waypoint;
		waypoint.header.frame_id = "base_footprint";
		waypoint.pose.position.x = 0.3;
		waypoint.pose.position.y = -0.7;
		waypoint.pose.position.y = 0.9; 
        */
		// Creating PoseStamped goal pose 
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = "odom";
        
        switch(detections[detection_index].id[0]){
        	case 1:
        		appro_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x;
				appro_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y;
				appro_pose.pose.position.z = detections[detection_index].pose.pose.pose.position.z + 0.20;  
				
				
				m.getRPY(roll, pitch, yaw);
				pitch = M_PI / 2;
				
				rotation_about_y.setRPY(roll, pitch, yaw);
				rotation_about_y.normalize();
				appro_pose.pose.orientation = tf2::toMsg(rotation_about_y);
				
        		goal_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x;
        		goal_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y;
        		goal_pose.pose.position.z = detections[detection_index].pose.pose.pose.position.z + 0.01; 
        		goal_pose.pose.orientation = appro_pose.pose.orientation;
        		
        		break;
        	case 2:
        		appro_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x + 0.015;
				appro_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y + 0.015;
				appro_pose.pose.position.z = detections[detection_index].pose.pose.pose.position.z + 0.20;  
				
				m.getRPY(roll, pitch, yaw);
				pitch = M_PI / 2;
				roll = 0.0;
				yaw += M_PI / 2;
				rotation_about_y.setRPY(roll, pitch, yaw);
				rotation_about_y.normalize();
				appro_pose.pose.orientation = tf2::toMsg(rotation_about_y);
				
        		goal_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x + 0.015;
        		goal_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y + 0.015;
        		goal_pose.pose.position.z =  0.755 + 0.13;
        		goal_pose.pose.orientation = appro_pose.pose.orientation;
        		break;
        	case 3:
        		appro_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x;
				appro_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y;
				appro_pose.pose.position.z = detections[detection_index].pose.pose.pose.position.z + 0.20;  
				
				m.getRPY(roll, pitch, yaw);
				pitch = M_PI / 2;

				rotation_about_y.setRPY(roll, pitch, yaw);
				rotation_about_y.normalize();
				appro_pose.pose.orientation = tf2::toMsg(rotation_about_y);
				
        		goal_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x;
        		goal_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y;
        		goal_pose.pose.position.z = 0.755 + 0.13; 
        		goal_pose.pose.orientation = appro_pose.pose.orientation;
        		break;
        	default:
        		ROS_ERROR("Error creating appro and goal pose");
        		break;
        }
        
        //waypoint.pose.orientation = appro_pose.pose.orientation;
		// Creating plan for appro
	    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	    
		//get home position to return in after picking
		geometry_msgs::PoseStamped home_pose;
		home_pose = gripper_group.getCurrentPose("gripper_grasping_frame");
		
		//add ollision objects to make tiago avoid them while reaching appro_pose
		addCollisionObjects(detections);
		
		arm_group.setPlannerId("SBLkConfigDefault");
		arm_group.setStartStateToCurrentState();
		arm_group.setMaxVelocityScalingFactor(1.0);
	    
	    arm_group.setPoseTarget(appro_pose, "gripper_grasping_frame"); //appro
	    //arm_group.setRPYTarget(0, -M_PI/4, 0, "gripper_grasping_frame_Z");
	    arm_group.setPlanningTime(7.0);
	    
	    bool success = bool(arm_group.plan(my_plan));

	    if(!success)
	        ROS_ERROR("No plan found for appro_pose");
		else{
	    	ROS_INFO_STREAM("Plan found for appro_pose in " << my_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = arm_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan appro_pose");
			else
				ROS_INFO_STREAM("Motion to appro_pose ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
		
		//Creating plan for goal
	    moveit::planning_interface::MoveGroupInterface::Plan my_plan_goal;
	    
	    //removing the pickable object from the planning_scene to make it pickable
	    std::vector<std::string> ids;
	    ids.push_back(std::to_string(detections[detection_index].id[0]));
		planning_scene_interface.removeCollisionObjects(ids);
		
		arm_group.setStartStateToCurrentState();
		arm_group.setMaxVelocityScalingFactor(1.0);
	    
	    arm_group.setPoseTarget(goal_pose, "gripper_grasping_frame"); //appro
	    arm_group.setPlanningTime(5.0);
	    
	    success = bool(arm_group.plan(my_plan_goal));

	    if(!success)
	        ROS_ERROR("No plan found for goal_pose");
		else{
	    	ROS_INFO_STREAM("Plan found for goal_pose in " << my_plan_goal.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = arm_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan goal_pose");
			else
				ROS_INFO_STREAM("Motion to goal_pose ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
				
		//close the gripper 
		moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
		
		gripper_group.setStartStateToCurrentState();
		gripper_group.setPlanningTime(2.0);
		
		std::vector<double> close_gripper_values;
		

		switch(detections[detection_index].id[0]){
			case 1:
				close_gripper_values.push_back(sqrt(2*(detections[detection_index].size[0]*detections[detection_index].size[0]))/2 - 0.005);
				close_gripper_values.push_back(sqrt(2*(detections[detection_index].size[0]*detections[detection_index].size[0]))/2 - 0.005);
				break;
			case 2:
				close_gripper_values.push_back((detections[detection_index].size[0])/2 - 0.005);
				close_gripper_values.push_back((detections[detection_index].size[0])/2 - 0.005);
				break;
			case 3:
				close_gripper_values.push_back((detections[detection_index].size[0])/2 - 0.005);
				close_gripper_values.push_back((detections[detection_index].size[0])/2 - 0.005);
    			break;
    		default:
    			ROS_ERROR("received object id to be attached represents an obstacle object");
    			return false;
    	}
    	
    	gripper_group.setJointValueTarget(close_gripper_values);
		
    	success = bool(gripper_group.plan(gripper_plan));

	    if(!success)
	        ROS_ERROR("No plan found for closing gripper");
		else{
	    	ROS_INFO_STREAM("Plan found for closing gripper in " << gripper_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = gripper_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan closing gripper");
			else
				ROS_INFO_STREAM("Motion to closing gripper ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
		
		std::vector<std::string> world_ids;
	    
		std::map<std::string, moveit_msgs::CollisionObject> map;
		moveit_msgs::CollisionObject object;
		
		
		switch(detections[detection_index].id[0]){
			case 1:
				world_ids.push_back("Hexagon");
				map = planning_scene_interface.getObjects(world_ids);
				object = map["Hexagon"];
				//gripper_group.attachObject("Hexagon", "gripper_grasping_frame");
				break;
			case 2:
				world_ids.push_back("Triangle");
				map = planning_scene_interface.getObjects(world_ids);
				object = map["Triangle"];
				//gripper_group.attachObject("Triangle", "gripper_grasping_frame");
				break;
			case 3:
				world_ids.push_back("cube");
				map = planning_scene_interface.getObjects(world_ids);
				object = map["cube"];
				//gripper_group.attachObject("cube", "gripper_grasping_frame");
    			break;
    		default:
    			ROS_ERROR("received object id to be attached represents an obstacle object");
    			return false;
    	}
    	/*
		
		*/
		
		moveit_msgs::AttachedCollisionObject attached_object;
		attached_object.link_name = "gripper_grasping_frame";
		attached_object.object = object;
    	planning_scene_interface.applyAttachedCollisionObject(attached_object);
    	
    	
    	//planning_scene_interface.applyPlanningScene();
    	
    	moveit::planning_interface::MoveGroupInterface::Plan my_plan_appro;
		
		arm_group.setPlannerId("SBLkConfigDefault");
		arm_group.setStartStateToCurrentState();
		arm_group.setMaxVelocityScalingFactor(1.0);
	    
	    arm_group.setPoseTarget(appro_pose, "gripper_grasping_frame"); //appro
	    arm_group.setPlanningTime(7.0);
	    
	    success = bool(arm_group.plan(my_plan_appro));

	    if(!success)
	        ROS_ERROR("No plan found for departs");
		else{
	    	ROS_INFO_STREAM("Plan found for departs in " << my_plan_appro.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = arm_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan departs");
			else
				ROS_INFO_STREAM("Motion to departs ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
		/*
		//plan to waypoint
		moveit::planning_interface::MoveGroupInterface::Plan waypoint_plan;
		
		arm_group.setStartStateToCurrentState();
		arm_group.setMaxVelocityScalingFactor(1.0);
	    
	    arm_group.setPoseTarget(waypoint, "gripper_grasping_frame"); //appro
	    arm_group.setPlanningTime(5.0);
	    
	    success = bool(arm_group.plan(waypoint_plan));

	    if(!success)
	        ROS_ERROR("No plan found for waypoint");
		else{
	    	ROS_INFO_STREAM("Plan found for waypoint in " << my_plan_goal.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = arm_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan waypoint");
			else
				ROS_INFO_STREAM("Motion to waypoint ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
		*/
		
		//plan to come back home
		moveit::planning_interface::MoveGroupInterface::Plan home_plan;
		
		arm_group.setPlannerId("SBLkConfigDefault");
		arm_group.setStartStateToCurrentState();
		arm_group.setPlanningTime(5.0);
		
		arm_group.setPoseTarget(home_pose, "gripper_grasping_frame"); //appro
		
    	success = bool(arm_group.plan(home_plan));

	    if(!success)
	        ROS_ERROR("No plan found for closing gripper");
		else{
	    	ROS_INFO_STREAM("Plan found to return to home configuration in " << home_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = arm_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan return to home configuration");
			else
				ROS_INFO_STREAM("Motion to return to home configuration ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
    	
        
        return true;

        // feedback_.status = 0;
        // as_.publishFeedback(feedback_);

        // feedback_.status = 1;
        // as_.publishFeedback(feedback_);
    }
    
    void addPlaceCollisionCylinder(std::vector<apriltag_ros::AprilTagDetection> detections, int correct_index){
    	std::vector<moveit_msgs::CollisionObject> collision_objects;
		
		moveit_msgs::CollisionObject place_cylinder;
		shape_msgs::SolidPrimitive primitive;
		geometry_msgs::Pose cylinder_pose;
		
		place_cylinder.id = "place_cylinder";
		primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
		place_cylinder.header.frame_id = "odom";
		
		primitive.dimensions.resize(2);
		primitive.dimensions[0] = 0.70;  // height
		primitive.dimensions[1] = 0.22;  // radius
		
		//ROS_INFO("Z dimension: %f", primitive.dimensions[0]); //height
		
		cylinder_pose.position.x = detections[correct_index].pose.pose.pose.position.x;
		cylinder_pose.position.y = detections[correct_index].pose.pose.pose.position.y;
		cylinder_pose.position.z = 0.35;
		cylinder_pose.orientation.x = 0.0;
		cylinder_pose.orientation.y = 0.0;
		cylinder_pose.orientation.z = 1.0;
		cylinder_pose.orientation.w = 0.0;

		place_cylinder.operation = 0; //ADD
		
		place_cylinder.primitives.push_back(primitive);
	    place_cylinder.primitive_poses.push_back(cylinder_pose);
	
	    collision_objects.push_back(place_cylinder);
		
		planning_scene_interface.applyCollisionObjects(collision_objects);
	}
    

    bool place(std::vector<apriltag_ros::AprilTagDetection> detections, int correct_index){
		
		geometry_msgs::PoseStamped place_pose;
        place_pose.header.frame_id = "odom";
        place_pose.pose.position.x = detections[correct_index].pose.pose.pose.position.x;
        place_pose.pose.position.y = detections[correct_index].pose.pose.pose.position.y;
        
        
		double roll = 0.0;
		double pitch = M_PI / 2;
		double yaw = 0.0;
		tf2::Quaternion rotation_about_y;
		rotation_about_y.setRPY(roll, pitch, yaw);
		rotation_about_y.normalize();
		place_pose.pose.orientation = tf2::toMsg(rotation_about_y);
        
        switch(detections[correct_index].id[0]){
        	case 1:
        		place_pose.pose.position.z = 0.69 + 0.02 + 0.2 + 0.04; 
        		break;
        	default:
        		place_pose.pose.position.z = 0.69 + 0.14; 
        		break;
        }
        
        //create place plan and execute associated motion
        moveit::planning_interface::MoveGroupInterface::Plan place_plan;
	  	
	  	
		//create place_cylinder collision object to be avoided during place motion
		addPlaceCollisionCylinder(detections, correct_index);
		
		
		//place the object
		arm_group.setPlannerId("SBLkConfigDefault");
		arm_group.setStartStateToCurrentState();
		arm_group.setMaxVelocityScalingFactor(1.0);
	    
	    //get home_pose to return after place action
	    geometry_msgs::PoseStamped home_pose;
		home_pose = gripper_group.getCurrentPose("gripper_grasping_frame");
		
		//set goal target
	    arm_group.setPoseTarget(place_pose, "gripper_grasping_frame"); //appro
	    arm_group.setPlanningTime(5.0);
	    
	    bool success = bool(arm_group.plan(place_plan));

	    if(!success)
	        ROS_ERROR("No plan found for place_pose");
		else{
	    	ROS_INFO_STREAM("Plan found for place_pose in " << place_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = arm_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan place_pose");
			else
				ROS_INFO_STREAM("Motion to place_pose ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
		
		//open gripper
		moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
		
		gripper_group.setStartStateToCurrentState();
		gripper_group.setPlanningTime(2.0);
		
		std::vector<double> open_gripper_values;
		open_gripper_values.push_back(0.023);
		open_gripper_values.push_back(0.023);
		gripper_group.setJointValueTarget(open_gripper_values);
		
    	success = bool(gripper_group.plan(gripper_plan));

	    if(!success)
	        ROS_ERROR("No plan found for opening gripper");
		else{
	    	ROS_INFO_STREAM("Plan found for opening gripper in " << gripper_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = gripper_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan opening gripper");
			else
				ROS_INFO_STREAM("Motion to opening gripper ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
		
		
		gripper_group.detachObject();
		
		//planning_scene_interface.applyPlanningScene();
		
		//come back to home_pose
		moveit::planning_interface::MoveGroupInterface::Plan home_plan;
		
		arm_group.setPlannerId("SBLkConfigDefault");
		arm_group.setStartStateToCurrentState();
		arm_group.setPlanningTime(5.0);
		
		arm_group.setPoseTarget(home_pose, "gripper_grasping_frame"); //appro
		
    	success = bool(arm_group.plan(home_plan));

	    if(!success)
	        ROS_ERROR("No plan found for closing gripper");
		else{
	    	ROS_INFO_STREAM("Plan found to return to home configuration in " << home_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = arm_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan return to home configuration");
			else
				ROS_INFO_STREAM("Motion to return to home configuration ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
	  	
        return true;
    }


    /**
     * @brief Callback function for executing the navigation action.
     * @param goal The goal for the pose action.
     */
    void executeCB(const assignment2::ArmGoalConstPtr &goal) { 
				
		bool objectPicked = false;
		bool objectPlaced = false;
		
        switch(goal->request){
            case 1:
                objectPicked = pick(goal->detections, goal->id);
                if (objectPicked){
                    result_.objectPicked = objectPicked;
                    as_.setSucceeded(result_);
                } else {
                    result_.objectPicked = objectPicked;
                    as_.setAborted(result_);
                }
                break;
            case 2:
                objectPlaced = place(goal->detections, goal->id);
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
