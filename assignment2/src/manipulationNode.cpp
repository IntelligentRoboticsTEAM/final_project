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
#include "utils.h"

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

public:

    ArmAction(std::string name) : as_(nh_, name, boost::bind(&ArmAction::executeCB, this, _1), false), action_name_(name)
    {
    	as_.start();
    }
    
    ~ArmAction(void){}
    
    void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::vector<apriltag_ros::AprilTagDetection> detections)
	{
	  std::vector<moveit_msgs::CollisionObject> collision_objects;

	  moveit_msgs::CollisionObject table_object;
	  table_object.id = "table";
	  table_object.header.frame_id = "odom";
	  
	  // Define the shape of the collision object
	  shape_msgs::SolidPrimitive primitive;
	  primitive.type = shape_msgs::SolidPrimitive::BOX;
	  primitive.dimensions.resize(3);
	  primitive.dimensions[0] = 0.92;  // x dimension
	  primitive.dimensions[1] = 0.92;  // y dimension
	  primitive.dimensions[2] = 0.755;  // z dimension

	  // Set the pose of the collision object in the odom frame
	  geometry_msgs::Pose table_pose;
	  table_pose.orientation.x = 0.0;
	  table_pose.orientation.y = 0.0;
	  table_pose.orientation.z = 1.0;
	  table_pose.orientation.w = 0.0;  // Quaternion identity
	  table_pose.position.x = 7.82;
	  table_pose.position.y = -2.98;
	  table_pose.position.z = 0.375;
	  
	  table_object.operation = 0; //ADD
	  
	  // Add the primitive to the collision object
	  table_object.primitives.push_back(primitive);
	  table_object.primitive_poses.push_back(table_pose); //map
	
	  collision_objects.push_back(table_object);

	  for(int i = 0; i < detections.size(); i++)
	  {

		moveit_msgs::CollisionObject obstacle_object;
		shape_msgs::SolidPrimitive primitive;
		geometry_msgs::Pose object_pose;
		
		switch((int)detections[i].id[0]){
			case 1:
				obstacle_object.id = std::to_string(detections[i].id[0]);
				primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
				obstacle_object.header.frame_id = "odom";
				
				primitive.dimensions.resize(2);
				primitive.dimensions[0] = detections[i].pose.pose.pose.position.z - 0.755;  // height
				primitive.dimensions[1] = sqrt(2*(detections[i].size[0]*detections[i].size[0]))/2 + 0.005;  // radius
				
				//ROS_INFO("Z dimension: %f", primitive.dimensions[0]); //height
				
				object_pose.position.x = detections[i].pose.pose.pose.position.x;
				object_pose.position.y = detections[i].pose.pose.pose.position.y;
				object_pose.position.z = detections[i].pose.pose.pose.position.z - primitive.dimensions[0] / 2;
				object_pose.orientation = detections[i].pose.pose.pose.orientation;
		
				obstacle_object.operation = 0; //ADD
				
				break;
			case 2: //da rivedere perche ha una forma strana
				obstacle_object.id = std::to_string(detections[i].id[0]);
				obstacle_object.header.frame_id = "odom";
				
				primitive.type = shape_msgs::SolidPrimitive::BOX;
				primitive.dimensions.resize(3);
				primitive.dimensions[0] = detections[i].size[0] + 0.015;  // x dimension
				primitive.dimensions[1] = detections[i].size[0] + 0.015;  // y dimension
				primitive.dimensions[2] = detections[i].size[0] + 0.015;  // z dimension
				
				//ROS_INFO("Z dimension: %f", primitive.dimensions[2]);
				
				object_pose.position.x = detections[i].pose.pose.pose.position.x + 0.015;
				object_pose.position.y = detections[i].pose.pose.pose.position.y + 0.015;
				object_pose.position.z = 0.755;
				object_pose.orientation = detections[i].pose.pose.pose.orientation;
				
				obstacle_object.operation = 0; //ADD
				
				break;
				
			case 3:
				obstacle_object.id = std::to_string(detections[i].id[0]);				
				obstacle_object.header.frame_id = "odom";
				
				primitive.type = shape_msgs::SolidPrimitive::BOX;
				primitive.dimensions.resize(3);
				primitive.dimensions[0] = detections[i].size[0] + 0.015;  // x dimension
				primitive.dimensions[1] = detections[i].size[0] + 0.015;  // y dimension
				primitive.dimensions[2] = detections[i].size[0] + 0.015;  // z dimension
				
				//ROS_INFO("Z dimension: %f", primitive.dimensions[2]);
				
				object_pose.position.x = detections[i].pose.pose.pose.position.x;
				object_pose.position.y = detections[i].pose.pose.pose.position.y;
				object_pose.position.z = detections[i].pose.pose.pose.position.z - primitive.dimensions[2] / 2;
				object_pose.orientation = detections[i].pose.pose.pose.orientation;
				
				obstacle_object.operation = 0; //ADD
				
				break;
				
			default:
				obstacle_object.id = std::to_string(detections[i].id[0]);				
				obstacle_object.header.frame_id = "odom";
				
				primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
				primitive.dimensions.resize(2);
				primitive.dimensions[0] = detections[i].pose.pose.pose.position.z - 0.755 + 0.1;  // height
				primitive.dimensions[1] = sqrt(2*(detections[i].size[0]*detections[i].size[0]))/2 + 0.02;  // radius
				
				//ROS_INFO("Z dimension: %f", primitive.dimensions[2]);
				
				object_pose.position.x = detections[i].pose.pose.pose.position.x;
				object_pose.position.y = detections[i].pose.pose.pose.position.y;
				object_pose.position.z = detections[i].pose.pose.pose.position.z - 0.1;// - primitive.dimensions[0] / 2;
				object_pose.orientation = detections[i].pose.pose.pose.orientation;
				
				obstacle_object.operation = 0; //ADD
				break;
		}
		
		obstacle_object.primitives.push_back(primitive);
		obstacle_object.primitive_poses.push_back(object_pose);
		
		collision_objects.push_back(obstacle_object);
		
	  }
	  
	  planning_scene_interface.applyCollisionObjects(collision_objects);
	}

	
    bool pickTutorial(std::vector<apriltag_ros::AprilTagDetection> detections, int requestedID){
    
		
		int detection_index = 0;
		  while(detections[detection_index].id[0] != requestedID){
		  	detection_index++;
		}
    	
    	// Creating PoseStamped approach/depart pose 
        geometry_msgs::PoseStamped appro_pose;
        appro_pose.header.frame_id = "odom";
        appro_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x;
        appro_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y;
        appro_pose.pose.position.z = detections[detection_index].pose.pose.pose.position.z + 0.20;  
        
          // Original quaternion
		tf::Quaternion original_quaternion(detections[detection_index].pose.pose.pose.orientation.x, detections[detection_index].pose.pose.pose.orientation.y, detections[detection_index].pose.pose.pose.orientation.z, detections[detection_index].pose.pose.pose.orientation.w);  // Replace with your quaternion values

		// Construct a quaternion representing a 90-degree rotation about the Y-axis
		tf::Quaternion rotation_about_y;
		rotation_about_y.setRPY(0.0, M_PI / 2.0, 0.0);  // 90 degrees rotation about Y-axis

		// Multiply the original quaternion by the rotation quaternion
		tf::Quaternion rotated_quaternion = rotation_about_y * original_quaternion;

		// Normalize the result to ensure it remains a unit quaternion
		rotated_quaternion.normalize();
		
		geometry_msgs::Quaternion geometry_msgs_quaternion;
    	tf::quaternionTFToMsg(rotated_quaternion, geometry_msgs_quaternion);
		
    	appro_pose.pose.orientation = geometry_msgs_quaternion;
    	/*
    	tf2::Quaternion q(appro_pose.pose.orientation.x, appro_pose.pose.orientation.y, appro_pose.pose.orientation.z, appro_pose.pose.orientation.w);  // Replace with your quaternion values

		// Convert quaternion to RPY
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		// Print the RPY angles
		ROS_INFO("appro_pose: Roll: %.2f, Pitch: %.2f, Yaw: %.2f\nwith respect to odom", roll, pitch, yaw);
    	*/
		//ROS_INFO("appro_orientation: %f \t %f \t %f \t %f", appro_pose.pose.orientation.x, appro_pose.pose.orientation.y, appro_pose.pose.orientation.z, appro_pose.pose.orientation.w);    	

    	//geometry_msgs::PoseStamped appro_pose_1;
        
		// Creating PoseStamped goal pose 
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = "odom";
        goal_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x;
        goal_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y;
        goal_pose.pose.position.z = detections[detection_index].pose.pose.pose.position.z - (detections[detection_index].pose.pose.pose.position.z - 0.755) / 2; 
        goal_pose.pose.orientation = detections[detection_index].pose.pose.pose.orientation;

  		
		// Creating plan
	    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		moveit::planning_interface::MoveGroupInterface group("arm_torso");
		addCollisionObjects(planning_scene_interface, detections);
		
		group.setPlannerId("SBLkConfigDefault");
		//group.setPoseReferenceFrame("odom");
		group.setStartStateToCurrentState();
		group.setMaxVelocityScalingFactor(1.0);
	    
	    group.setPoseTarget(appro_pose, "gripper_grasping_frame"); //appro
	    //group.setRPYTarget(0, -M_PI/4, 0, "gripper_grasping_frame_Z");
	    group.setPlanningTime(10.0);
	    
	    bool success = bool(group.plan(my_plan));

	    if(!success)
	        ROS_ERROR("No plan found for appro_pose");
		else{
	    	ROS_INFO_STREAM("Plan found for appro_pose in " << my_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan appro_pose");
			else
				ROS_INFO_STREAM("Motion to appro_pose ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
		
        return true;

        // feedback_.status = 0;
        // as_.publishFeedback(feedback_);

        // feedback_.status = 1;
        // as_.publishFeedback(feedback_);
    }

    bool placeObject(std::vector<apriltag_ros::AprilTagDetection> detections){

        //spinner.stop();
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
                objectPicked = pickTutorial(goal->detections, goal->id);
                if (objectPicked){
                    result_.objectPicked = objectPicked;
                    as_.setSucceeded(result_);
                } else {
                    result_.objectPicked = objectPicked;
                    as_.setAborted(result_);
                }
                break;
            case 2:
                objectPlaced = placeObject(goal->detections);
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
