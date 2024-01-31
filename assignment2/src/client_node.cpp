// ROS libraries
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// ROS messages and services
#include <geometry_msgs/Pose.h>
#include <tiago_iaslab_simulation/Objs.h>
// Custom headers
#include <assignment2/PoseAction.h>
#include <assignment2/ArmAction.h>
#include <assignment2/Scan.h>
#include "headers/utils.h"
#include "headers/navigation_methods.h"


#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>


const float x_offset = 0.1;
const float y_offset = 0.5;

template <typename Action>
bool isServerAvailable(const actionlib::SimpleActionClient<Action>& client, const std::string& serverName);
void feedbackNavigation(const assignment2::PoseFeedbackConstPtr& feedback);
void feedbackManipulation(const assignment2::ArmFeedbackConstPtr& feedback);
int doNavigation(int goalChoice, int object_order, actionlib::SimpleActionClient<assignment2::PoseAction> &acNavigation, const apriltag_ros::AprilTagDetection &scanResponse);
int doPick(int object_order, ros::ServiceClient &detectionClient , actionlib::SimpleActionClient<assignment2::ArmAction> &acManipulation);
int doScan(ros::ServiceClient &scan_client, std::vector<apriltag_ros::AprilTagDetection> &scanResponse, boost::shared_ptr<const sensor_msgs::LaserScan> msg);
int doPlace(int object_order, std::vector<apriltag_ros::AprilTagDetection> tempResponses, actionlib::SimpleActionClient<assignment2::ArmAction> &acManipulation);
bool doRecoveryNavigation(int object_order, actionlib::SimpleActionClient<assignment2::PoseAction> &acNavigation);

int main(int argc, char **argv) {

	

    //declaring node name
    ros::init(argc, argv, "client_pose_revisited");
    ros::NodeHandle nh;
    
    /*
     	0) Human Node Service
   	*/
    ros::ServiceClient human_client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
    tiago_iaslab_simulation::Objs human_srv;
    human_srv.request.ready = true;
    human_srv.request.all_objs = true;
    
    //vector to store object order
    std::vector<int> object_order;
    
    if(human_client.call(human_srv)){
    	for(int i = 0; i < human_srv.response.ids.size(); i++)
    	{
    		object_order.push_back(human_srv.response.ids[i]);  //populate the vector previously defined with service response
    		ROS_INFO("Object ID: %d", (int)human_srv.response.ids[i]);
    	}
    }
    else{
    	//if the service fails we shutdown all ros nodes, unable to get object order
    	ROS_ERROR("Failed to call service to get pick object order");
    	ros::shutdown();
        return 1;
    }
    
    /////// ------ ///////
    // DELCARATIONS
   
    //Simple Action Clients
    	// Navigation client
    actionlib::SimpleActionClient<assignment2::PoseAction> acNavigation("poseRevisited", true);
    if (!isServerAvailable(acNavigation, "Navigation")) return 1;
        //Manipulation client
    actionlib::SimpleActionClient<assignment2::ArmAction> acManipulation("manipulationNode", true);
    if (!isServerAvailable(acNavigation, "Manipulation")) return 1;
    
	// Service CLients
   		//Detection service client
    ros::ServiceClient detectionClient = nh.serviceClient<assignment2::Detection>("/object_detection");
    	//Scan clients
	ros::ServiceClient scan_client = nh.serviceClient<assignment2::Scan>("/scan_node");
    
    // Other Variables Declarations
	boost::shared_ptr<const sensor_msgs::LaserScan> msg;	// This will store the laser scan message
	apriltag_ros::AprilTagDetection nullAprilTag;			// This will work as an empty struct to pass in order to use doNavigation() function when the AprilTag parameter is not used
	std::vector<int> ids;									// This will contain the ids in the correct order from right to left
	int returnVal;											// This variable will contain the return value of every function. When an error occurs, this is set to 1 and the program is shutdown.
	std::vector<apriltag_ros::AprilTagDetection> scanResponse; // This vector will contain the Positions and the Colors of the scanned cylinders
	

	/////// ------ ///////
    // EXECUTION
	
	/*
		1) go to table (with harcoded position relatives to objects)
	*/ 
	returnVal = doNavigation(1, object_order[0], acNavigation, nullAprilTag);
	if(returnVal == 1) return 1;
	else returnVal = 0;
	
	for(int i = 0; i < object_order.size(); i++){
	
		int order = object_order[i];
		int nextOrder = object_order[i+1];
		
	/*
		2) Pick the object
	*/
		returnVal = doPick(object_order[i], detectionClient, acManipulation);
		if(returnVal == 1) return 1;
		else returnVal = 0;

	/*
		3) Go the Scan position 
	*/ 
		returnVal = doNavigation(2, order, acNavigation, nullAprilTag);
		if(returnVal == 1) return 1;
		else returnVal = 0;
		
		
	/*
		4) Scan the cylinders 
		
		This only happens once. It is not necessary to always recompute these results.
		The invoked function will scan the positions of the cylinders and will be able to recognize the colors of the cylinders using OPENCV.
		It will return scanResponse, that associates the right IDs (of color) with the right Positions of the cylinders.
	*/ 
		if(i == 0){		
			msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);
			returnVal = doScan(scan_client, scanResponse, msg);
			if(returnVal == 1) return 1;
			else returnVal = 0;
			
			for(int i = 0; i < scanResponse.size(); i++){
				ids.push_back(scanResponse[i].id[0]);
			}
	 		
	 	}
	 	
		int correct_index;
		for(int k = 0; k < ids.size(); k++)
		{
			if(ids[k] == order)
				correct_index = k;
		} 
		
	/*
		5) Go the cylinder
	*/ 
	
		apriltag_ros::AprilTagDetection tempResponse;
	
	
	// !!!!!!!!!!.......... CHANGE != WITH == ..........!!!!!!!!!!
		if(ids.size() == 3 && scanResponse.size() == 3)
		{
			tempResponse.pose.pose.pose.position.x = scanResponse[correct_index].pose.pose.pose.position.x - x_offset;
			tempResponse.pose.pose.pose.position.y = scanResponse[correct_index].pose.pose.pose.position.y - y_offset;
			tempResponse.pose.pose.pose.position.z = 0.00;
			tempResponse.id.push_back(correct_index);
			
			returnVal = doNavigation(3, order, acNavigation, tempResponse);
			if(returnVal == 1) return 1;
			else returnVal = 0;
		} 
		else 
		{
			returnVal = doRecoveryNavigation(order, acNavigation);
			if(returnVal == 1) return 1;
			else returnVal = 0;
		}	
		
	/*
		6) Place the object on the cylinder
	*/ 	
		
		returnVal = doPlace(correct_index, scanResponse, acManipulation);
		if(returnVal == 1) return 1;
		else returnVal = 0;
	
	/*				
	
		7) Go back to Home position and restart the cycle (when i < 2)
		   Go back to Home position (when i = 2)
			
	*/ 	
		if(i < 2){
			returnVal = doNavigation(4, nextOrder, acNavigation,  nullAprilTag);
			if(returnVal == 1) return 1;
			else returnVal = 0;
		} 
		
		
		if (i == 2){
			returnVal = doNavigation(5, order, acNavigation,  nullAprilTag);
			if(returnVal == 1) return 1;
			else returnVal = 0;
		}
	}

    return 0;
}

/**
 * @brief This method moves tiago to the desired position, depending on the value of goalChoice and object_order
 * @param goalChoice for value = 1: tiago moves to the table, for value = 2: tiago moves to scan position, for value = 3: tiago moves to the cylinder, for value = 4: tiago moves to home and then to the table, for value = 5: tiago moves to home position
 * @param object_order id received from human node
 * @param acNavigation action client that manages the navigation
 * @param scanResponse output of scan operation needed to move to right cylinder (for goalChoice = 3)
 * @return 1 if the robot reaches the goal within the timeout and 0 otherwise.
 */
int doNavigation(int goalChoice, int object_order, actionlib::SimpleActionClient<assignment2::PoseAction> &acNavigation, const apriltag_ros::AprilTagDetection &scanResponse)
{
	//goal pose to navigation 
	assignment2::PoseGoal navigation_goal;
	//goalChoice is used to manage the different pieces of navigation (go to pick/place table, go to waypoint, ecc) 
	navigation_goal.operation = goalChoice;
	//object order is the the object's id order
	navigation_goal.id = object_order;
	
	//in the second choice we need to pass the place-cylinder pose
	if(goalChoice == 3 ){
		navigation_goal.detection = scanResponse;
	}
	
	acNavigation.sendGoal(navigation_goal, NULL, NULL, &feedbackNavigation);
	bool finished_before_timeout = acNavigation.waitForResult(ros::Duration(60.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state_final = acNavigation.getState();
		ROS_INFO("Action finished: %s", state_final.toString().c_str());
		if (state_final == actionlib::SimpleClientGoalState::SUCCEEDED) {
		    	ROS_INFO("Navigation to goal finished before timeout");
		}
	} else {
		ROS_ERROR("Navigation to goal did not finish before the timeout.");
		acNavigation.cancelGoal();
		ROS_ERROR("Goal has been cancelled");
		return 1;
	}
	
	return 0;
}

/**
 * @brief This method makes a scan to find the position of the cylinders plus their associated colores
 * @param scan_client service client called in the method
 * @param scanResponse object that stores cylinders' colors and poses
 * @param msg variable that contains the output from laser scan
 * @return 1 if the robot scans correctly and 0 otherwise.
 */
int doScan(ros::ServiceClient &scan_client, std::vector<apriltag_ros::AprilTagDetection> &scanResponse, boost::shared_ptr<const sensor_msgs::LaserScan> msg)
{   
	assignment2::Scan srv2;
 	srv2.request.msg = *msg; // LASER SCAN
    srv2.request.ready = true; 
    
    if(scan_client.call(srv2))
    {        
        for(int i=0; i<srv2.response.poses.size(); i++)
        {
            apriltag_ros::AprilTagDetection cylinder_pose;
            
            cylinder_pose.pose.pose.pose = srv2.response.poses[i];
            cylinder_pose.id.push_back(srv2.response.ids_associated_colors[i]);
            
            scanResponse.push_back(cylinder_pose);
        }
    }
    else
    {
        ROS_ERROR("Failed to call Scan Service");
        return 1;
    }
    
    return 0;
}

/**
 * @brief This method picks the required object
 * @param object_order id received from human node
 * @param detectionClient client called in the method
 * @param acManipulation action client that manages the manipulation
 * @return 1 if the robot picks the object correctly and 0 otherwise.
 */
int doPick(int object_order, ros::ServiceClient &detectionClient , actionlib::SimpleActionClient<assignment2::ArmAction> &acManipulation)
{
	assignment2::Detection detection_srv;
    detection_srv.request.ready = true;
    detection_srv.request.requested_id = object_order;
    
    //create manipulation goal and client
	assignment2::ArmGoal armGoal;
	
	//if the service call was successful
    if(detectionClient.call(detection_srv)){        
        //construct goal for manipulation node
        armGoal.request = 1; 
        armGoal.id = object_order;
        armGoal.detections = detection_srv.response.detections;
      	
      	acManipulation.sendGoal(armGoal, NULL, NULL, &feedbackManipulation);
		
		bool manipulation_finished_before_timeout = acManipulation.waitForResult(ros::Duration(60.0));
    
		if (manipulation_finished_before_timeout) {
		    actionlib::SimpleClientGoalState state = acManipulation.getState();
		    ROS_INFO("Action finished: %s", state.toString().c_str());
		    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
				ROS_INFO("Pick done");
		    }
		} else {
		    ROS_ERROR("Pick action did not finish before the timeout.");
		    acManipulation.cancelGoal();
		    ROS_ERROR("Pick goal has been cancelled");
		}
      	
    }else{
    	ROS_ERROR("Detection service failed");
    	ros::shutdown();
		return 1;
    }
    return 0;
}

/**
 * @brief This method places the required object
 * @param object_order id received from human node
 * @param tempResponses object that contains cylinders' colors and poses
 * @param acManipulation action client that manages the manipulation
 * @return 1 if the robot places the object correctly and 0 otherwise.
 */
int doPlace(int object_order, std::vector<apriltag_ros::AprilTagDetection> tempResponses, actionlib::SimpleActionClient<assignment2::ArmAction> &acManipulation)
{   
	assignment2::ArmGoal armGoal;

  	// Goal for Manipulation
    armGoal.request = 2; 
    armGoal.id = object_order;
    armGoal.detections = tempResponses;
  	
  	acManipulation.sendGoal(armGoal, NULL, NULL, &feedbackManipulation);
	
	bool manipulation_finished_before_timeout = acManipulation.waitForResult(ros::Duration(60.0));

	if (manipulation_finished_before_timeout) {
	    actionlib::SimpleClientGoalState state = acManipulation.getState();
	    ROS_INFO("Action finished: %s", state.toString().c_str());
	    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("Place done");
	    }
	} else {
	    ROS_ERROR("Place action did not finish before the timeout.");
	    acManipulation.cancelGoal();
	    ROS_ERROR("Place goal has been cancelled");
	}
	
    return 0;
}

/**
 * @brief Callback function to handle feedback from the manipulation server.
 * @param feedback The feedback received from the server.
 */
void feedbackManipulation(const assignment2::ArmFeedbackConstPtr& feedback) {
    int status = feedback->status;

    switch (status) {
        case 0:
            ROS_INFO("Received status: Pick Started");
            break;
        case 1:
            ROS_INFO("Received status: Place Started");
            break;
        case 2:
            ROS_INFO("Received status: Gripper is Open");
            break;
        case 3:
            ROS_INFO("Received status: Gripper is Closed");
            break;
        case 4:
            ROS_INFO("Received status: Arm is moving");
            break;
        case 5:
            ROS_INFO("Received status: Object picked");
            break;
        case 6:
            ROS_INFO("Received status: Object placed");
            break;
		case 7:
			ROS_INFO("Received status: Arm is closed");
			break;
        default:
            ROS_INFO("Received unknown status");
            break;
    }
}

/**
 * @brief Callback function to handle feedback from the PoseAction server.
 * @param feedback The feedback received from the server.
 */
 void feedbackNavigation(const assignment2::PoseFeedbackConstPtr& feedback) {
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


template <typename Action>
/**
 * @brief Shuts down all the nodes if some errors occurr when calling manipulation or navigation
 * @param client indicates if the problem comes from manipulation or navigation
 * @param serverName string passed to print
 * @return true if executes correctly, false otherwise
 */
bool isServerAvailable(const actionlib::SimpleActionClient<Action>& client, const std::string& serverName){
    ROS_INFO("Waiting for %s server to start.", serverName.c_str());

    if (!client.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("%s server not available, shutting down", serverName.c_str());
        ros::shutdown();
        return false;
    }

    ROS_INFO("%s server started", serverName.c_str());
    return true;
}

/**
 * @brief if scan of cylinders is not executed correctly, this method makes tiago follow a recovery plan for navigation to hard-coded positions
 * @param object_order id received from human node
 * @param acNavigation action client that manages the navigation
 */
bool doRecoveryNavigation(int object_order, actionlib::SimpleActionClient<assignment2::PoseAction> &acNavigation){

	ROS_WARN("RECOVERY PLAN IN USE FOR NAVIGATION!");
	
	geometry_msgs::Pose bluePose;	geometry_msgs::Pose greenPose; 	geometry_msgs::Pose redPose;
	bluePose.position.x = 12.4; 	bluePose.position.y = -0.85;	bluePose.position.z = 0;
	greenPose.position.x = 11.4;	greenPose.position.y = -0.85;	greenPose.position.z = 0;
	redPose.position.x = 10.4;		redPose.position.y = -0.85;		redPose.position.z = 0;
	
	apriltag_ros::AprilTagDetection tempResponse;
	
	switch(object_order){
		case 1: 
			tempResponse.pose.pose.pose = bluePose;
			break;
		case 2: 
			tempResponse.pose.pose.pose = greenPose;
			break;
		case 3: 
			tempResponse.pose.pose.pose = redPose;
			break;
		default:
			ROS_ERROR("ERROR IN RECOVERY NAVIGATION TO CYLINDERS. ORDER = %d", object_order);
			return false;
	}
	
	tempResponse.id.push_back(object_order);
	
	return doNavigation(3, object_order, acNavigation, tempResponse );
}




