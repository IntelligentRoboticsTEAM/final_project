#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment2/PoseAction.h>
#include <assignment2/ArmAction.h>
#include <assignment2/Scan.h> 
#include <geometry_msgs/Pose.h>
#include <tiago_iaslab_simulation/Objs.h>
#include "headers/utils.h"
#include "headers/navigation_methods.h"


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
        case 5:
        	ROS_INFO("Going to WAYPOINT");
        default:
            ROS_INFO("Received unknown status");
            break;
    }
}
 
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
            ROS_INFO("Received status: Arm is High");
            break;
        case 5:
            ROS_INFO("Received status: Arm is Low");
            break;
        case 6:
            ROS_INFO("Received status: Action Ended");
            break;
        default:
            ROS_INFO("Received unknown status");
            break;
    }
}


int doNavigation(int goalChoice, int object_order, actionlib::SimpleActionClient<assignment2::PoseAction> &acNavigation, const apriltag_ros::AprilTagDetection &scanResponse)
{
	//goal pose to navigation 
	assignment2::PoseGoal navigation_goal;
	//goalChoice is used to manage the different pieces of navigation (go to pick/place table, go to waypoint, ecc) 
	navigation_goal.operation = goalChoice;
	//object order is the the object's id order
	navigation_goal.id = object_order;
	
	//in the second choice we need to pass the place-cylinder pose
	if(goalChoice == 2){
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
		ROS_INFO("Navigation to goal did not finish before the timeout.");
		acNavigation.cancelGoal();
		ROS_INFO("Goal has been cancelled");
		return 1;
	}
	
	return 0;
}

int doScan(ros::ServiceClient &scan_client, ros::ServiceClient &image_scan_client, std::vector<apriltag_ros::AprilTagDetection> &scanResponse, boost::shared_ptr<const sensor_msgs::LaserScan> msg)
{   
	assignment2::Scan srv2;
 	srv2.request.msg = *msg;
    srv2.request.ready = true; 
    
    if(scan_client.call(srv2))
    {
        ROS_INFO("Service call successful. Number of poses: %zu", srv2.response.poses.size());
        for(int i=0; i<srv2.response.poses.size(); i++)
        {
            apriltag_ros::AprilTagDetection cylinder_pose;
            cylinder_pose.pose.pose.pose = srv2.response.poses[i];
            scanResponse.push_back(cylinder_pose);
        }

    }
    else
    {
        ROS_ERROR("Failed to call Scan Service");
        return 1;
    }
    
    assignment2::Scan srv3;
	srv3.request.ready = true; 
	
	if(image_scan_client.call(srv3))
    {
		ROS_INFO("Service call successful. Number of ids: %zu", srv3.response.ids_associated_colors.size());
		ROS_INFO("Color Order: ");
		for (int i=0; i < srv3.response.ids_associated_colors.size(); i++)
		{	
			ROS_INFO("Color %d: %d", i+1, srv3.response.ids_associated_colors[i]);
			scanResponse[i].id.push_back(srv3.response.ids_associated_colors[i]);
		}
    }
    else
    {
        ROS_ERROR("Failed to call service, Client3");
        return 1;
    }

    /*for (int i = 0; i < std::min(srv3.response.ids_associated_colors.size(), srv2.response.poses.size()); ++i) {
        PoseID poseID;
        poseID.id = srv3.response.ids_associated_colors[i];
        poseID.pose.x = srv2.response.poses[i].position.x;
        poseID.pose.y = srv2.response.poses[i].position.y;

        poseIDVector.push_back(poseID);
        
        ROS_INFO("ID: %d, Position: (%.2f, %.2f)", poseID.id, poseID.pose.x, poseID.pose.y);
    }*/
    
    return 0;
}

int doPick(int object_order,  ros::ServiceClient &detection_client, actionlib::SimpleActionClient<assignment2::ArmAction> &acManipulation)
{
	assignment2::Detection detection_srv;
    detection_srv.request.ready = true;
    detection_srv.request.requested_id = object_order;
    
    //create manipulation goal and client
	assignment2::ArmGoal armGoal;
	std::vector<apriltag_ros::AprilTagDetection> detectionsObj;
	
	//if the service call was successful
    if(detection_client.call(detection_srv)){
        ROS_INFO("Detection done, tag id returned in base_footprint reference frame");
        
        //construct goal for manipulation node
        armGoal.request = 1; // PICK action is called
        armGoal.id = object_order;
        detectionsObj = detection_srv.response.detections;  
        armGoal.detections = detectionsObj;
      	
      	acManipulation.sendGoal(armGoal, NULL, NULL, &feedbackManipulation);
		
		bool manipulation_finished_before_timeout = acManipulation.waitForResult(ros::Duration(60.0));
    
		if (manipulation_finished_before_timeout) {
		    actionlib::SimpleClientGoalState state = acManipulation.getState();
		    ROS_INFO("Action finished: %s", state.toString().c_str());
		    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
				ROS_INFO("Pick done");
		    }
		} else {
		    ROS_INFO("Pick action did not finish before the timeout.");
		    acManipulation.cancelGoal();
		    ROS_INFO("Pick goal has been cancelled");
		}
      	
    }else{
    	ROS_ERROR("Detection service failed");
    	ros::shutdown();
		return 1;
    }
    return 0;
}


int doPlace(){
	return 0;
}

int main(int argc, char **argv) {
    
    //declaring node name
    ros::init(argc, argv, "client_pose_revisited");
    
    //nodeHandle to manage ros nodes connections
    ros::NodeHandle nh;
    
    //create service client to get pick object order
    ros::ServiceClient human_client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
    
    //send the request to human_node
    tiago_iaslab_simulation::Objs human_srv;
    human_srv.request.ready = true;
    human_srv.request.all_objs = true;
    
    //vector to store object order
    std::vector<int> object_order;
    
    //check if the service is done correctly
    if(human_client.call(human_srv)){
    	for(int i = 0; i < human_srv.response.ids.size(); i++){
    		//populate the vector previously defined with service response
    		object_order.push_back(human_srv.response.ids[i]);
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
    // CLIENT DELCARATIONS
    
    //navigation client
    actionlib::SimpleActionClient<assignment2::PoseAction> acNavigation("poseRevisited", true);
    
    //check if server is up for 5 seconds
    if (!acNavigation.waitForServer(ros::Duration(5.0))) { 
		ROS_ERROR("Navigation server not available, shutting down...");
		ros::shutdown();
		return 1;
	}
	ROS_INFO("Navigation server started");
	
    //Detection service client
    ros::ServiceClient detection_client = nh.serviceClient<assignment2::Detection>("/object_detection");
    
    //Manipulation client
    actionlib::SimpleActionClient<assignment2::ArmAction> acManipulation("manipulationNode", true);
    
    //check if server is up for 5 seconds
    if (!acNavigation.waitForServer(ros::Duration(5.0))) { 
		ROS_ERROR("Manipulation server not available, shutting down");
		ros::shutdown();
		return 1;
	}
	ROS_INFO("Manipulation server started");
	
    //Scan service client
	ros::ServiceClient scan_client = nh.serviceClient<assignment2::Scan>("/scan_node");
    
    //ImageScan service client
    ros::ServiceClient image_scan_client = nh.serviceClient<assignment2::Scan>("/image_colors_node");
	
	boost::shared_ptr<const sensor_msgs::LaserScan> msg;

	/////// ------ ///////
	//EXECUTION 
	
	apriltag_ros::AprilTagDetection nullAprilTag;
	int returnVal;
	
	for(int i = 0; i < object_order.size(); i++){
		
		//go to table (with harcoded position relatives to objects)
		returnVal = doNavigation(1, object_order[i], acNavigation, nullAprilTag);
		if(returnVal == 1) return 1;
		else returnVal = 0;

		//////// ------ ////////
		// PICK
		returnVal = doPick(object_order[i], detection_client, acManipulation);
		if(returnVal == 1) return 1;
		else returnVal = 0;
		
		/////// ------- ////////
		// SECOND NAVIGATION
		//go to cylinder scan position
		returnVal = doNavigation(2, object_order[i], acNavigation, nullAprilTag);
		if(returnVal == 1) return 1;
		else returnVal = 0;
		
		/////// ------- ////////
		// SCAN OF CYLINDERS
		std::vector<apriltag_ros::AprilTagDetection> scanResponse;
		
		if(i == 0){
			msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);
	 	}
	 	
	 	returnVal = doScan(scan_client, image_scan_client, scanResponse, msg);
	 	if(returnVal == 1) return 1;
		else returnVal = 0;
		
	   	/////// ------- ////////
		// FINAL NAVIGATION
		int correct_index;
		for(int k = 0; k < scanResponse.size(); k++){
			if(scanResponse[k].id[0] == object_order[i]){
				correct_index = k;
			}
		}
		
		returnVal = doNavigation(3, object_order[i], acNavigation, scanResponse[correct_index]);
		ROS_INFO("Object to pick: %d", (int)object_order[i]);
		ROS_INFO("Cylinder to go: %d", (int)scanResponse[correct_index].id[0]);
		ROS_INFO("X: %f\tY:%f\tZ:%f", (float)scanResponse[correct_index].pose.pose.pose.position.x, (float)scanResponse[correct_index].pose.pose.pose.position.y, (float)scanResponse[correct_index].pose.pose.pose.position.z);
		
		if(returnVal == 1) return 1;
		else returnVal = 0;
		
	 	/////// ------- ////////
		// PLACE
		
		returnVal = doPlace();
		if(returnVal == 1) return 1;
		else returnVal = 0;
		
	}
    
	
	
	
    
    return 0;
}

