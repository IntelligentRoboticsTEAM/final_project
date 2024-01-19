#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment2/PoseAction.h>
#include <assignment2/ArmAction.h>
#include <assignment2/Scan.h> 
#include <geometry_msgs/Pose.h>
#include <tiago_iaslab_simulation/Objs.h>
#include "utils.h"
#include "navigation_methods.h"


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
            ROS_INFO("Received status: Hand is Open");
            break;
        case 3:
            ROS_INFO("Received status: Hand is Closed");
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
	assignment2::PoseGoal goalFinal;
	goalFinal.operation = goalChoice; 	
	goalFinal.id = object_order;  
		
	if(goalChoice == 2){	
		goalFinal.detection = scanResponse;
	}
	
	/* if(goalChoice == 3){
		goalFinal
	} */
	
	acNavigation.sendGoal(goalFinal, NULL, NULL, &feedbackNavigation);
	bool finished_before_timeout_final = acNavigation.waitForResult(ros::Duration(60.0));

	if (finished_before_timeout_final) {
		actionlib::SimpleClientGoalState state_final = acNavigation.getState();
		ROS_INFO("Action finished: %s", state_final.toString().c_str());
		if (state_final == actionlib::SimpleClientGoalState::SUCCEEDED) {
		    	const auto& result_final = *acNavigation.getResult();
		    	//srv2.request.msg = result_final.msg;
		}
	} else {
		ROS_INFO("Action did not finish before the timeout.");
		acNavigation.cancelGoal();
		ROS_INFO("Goal has been cancelled");
		return 1;
	}
	
	return 0;
}

int doScan( ros::ServiceClient &scan_client, ros::ServiceClient &image_scan_client, std::vector<apriltag_ros::AprilTagDetection> &scanResponse, boost::shared_ptr<const sensor_msgs::LaserScan> msg)
{   
	assignment2::Scan srv2;
 	srv2.request.msg = *msg;
    srv2.request.ready = true; 
    
    if (scan_client.call(srv2))
    {
        ROS_INFO("Service call successful. Number of poses: %zu", srv2.response.poses.size());
        for(int i=0; i<srv2.response.poses.size(); i++)
        {
            scanResponse[i].pose.pose.pose = srv2.response.poses[i];
        }

    }
    else
    {
        ROS_ERROR("Failed to call Scan Service");
        return 1;
    }
    
    assignment2::Scan srv3;
	srv3.request.ready = true; 
	
	if (image_scan_client.call(srv3))
    {
		ROS_INFO("Service call successful. Number of ids: %zu", srv3.response.ids_associated_colors.size());
		ROS_INFO("Color Order: ");
		for (int i=0; i < srv3.response.ids_associated_colors.size(); i++)
		{
			ROS_INFO("Color %d: %d", i+1, srv3.response.ids_associated_colors[i]);
			scanResponse[i].id[0] = srv3.response.ids_associated_colors[i];
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

int doPick(const std::vector<int> &object_order,  ros::ServiceClient &detection_client, actionlib::SimpleActionClient<assignment2::ArmAction> &acManipulation)
{
	assignment2::Detection detection_srv;
    detection_srv.request.ready = true;
    detection_srv.request.requested_id = object_order[0];
    
    //create manipulation goal and client
	assignment2::ArmGoal armGoal;
	std::vector<apriltag_ros::AprilTagDetection> detectionsObj;
	
	//if the service call was successful
    if(detection_client.call(detection_srv)){
        ROS_INFO("Detection done, tag id returned in base_footprint reference frame");
        
        //construct goal for manipulation node
        armGoal.request = 1; // PICK action is called
        armGoal.id = object_order[0];
        detectionsObj = detection_srv.response.detections;  
        armGoal.detections = detectionsObj;
      	
      	acManipulation.sendGoal(armGoal, NULL, NULL, &feedbackManipulation);
		
		bool manipulation_finished_before_timeout = acManipulation.waitForResult(ros::Duration(60.0));
    
		if (manipulation_finished_before_timeout) {
		    actionlib::SimpleClientGoalState state = acManipulation.getState();
		    ROS_INFO("Action finished: %s", state.toString().c_str());
		    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
		        const auto& result = *acManipulation.getResult();        
		    }
		} else {
		    ROS_INFO("Manipulation action did not finish before the timeout.");
		    acManipulation.cancelGoal();
		    ROS_INFO("Manipulation goal has been cancelled");
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
    
    ros::init(argc, argv, "client_pose_revisited");
    
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
    
    tiago_iaslab_simulation::Objs srv;
    srv.request.ready = true;
    srv.request.all_objs = true;
    
    std::vector<int> object_order;
    
    if(client.call(srv)){
    	for(int i = 0; i < srv.response.ids.size(); i++){
    		object_order.push_back(srv.response.ids[i]);
    		ROS_INFO("Object ID: %d", (int)srv.response.ids[i]);
    	}
    }
    else{
    	ROS_ERROR("Failed to call service to get object sequence");
    	ros::shutdown();
        return 1;
    }
    
    /////// ------ ///////
    // CLIENT DELCARATIONS
    //1st navigation
    actionlib::SimpleActionClient<assignment2::PoseAction> acNavigation("poseRevisited", true);
    
    if (!acNavigation.waitForServer(ros::Duration(5.0))) { // Wait for 5 seconds
		ROS_ERROR("Action server not available");
		ros::shutdown();
		return 1;
	}
	ROS_INFO("Navigation Action server started.");
	
    //Detection
    ros::ServiceClient detection_client = nh.serviceClient<assignment2::Detection>("/object_detection");
    //Manipulation
    actionlib::SimpleActionClient<assignment2::ArmAction> acManipulation("manipulationNode", true);
    
    if (!acNavigation.waitForServer(ros::Duration(5.0))) { // Wait for 5 seconds
		ROS_ERROR("Action server not available");
		ros::shutdown();
		return 1;
	}
	ROS_INFO("Manipulation Action server started.");
	
    //Scan
	ros::ServiceClient scan_client = nh.serviceClient<assignment2::Scan>("/scan_node");
    //ImageScan
    ros::ServiceClient image_scan_client = nh.serviceClient<assignment2::Scan>("/image_colors_node");
    //Final Navigation
	
	//Place
	
	//GoBack Navigation


	
    apriltag_ros::AprilTagDetection nullAprilTag;

    //////// ------ ////////
    //  FIRST NAVIGATION
	int returnVal = doNavigation(1, object_order[0], acNavigation, nullAprilTag);
    if(returnVal == 1) return 1;
    else returnVal = 0;

    //////// ------ ////////
	// PICK
    returnVal = doPick(object_order, detection_client, acManipulation);
    if(returnVal == 1) return 1;
    else returnVal = 0;
	
	/////// ------- ////////
    // SECOND NAVIGATION
    returnVal = doNavigation(2, object_order[0], acNavigation, nullAprilTag);
    if(returnVal == 1) return 1;
    else returnVal = 0;
    
    /////// ------- ////////
    // SCAN OF CYLINDERS
    std::vector<apriltag_ros::AprilTagDetection> scanResponse;
    boost::shared_ptr<const sensor_msgs::LaserScan> msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);
 	returnVal = doScan(scan_client, image_scan_client, scanResponse, msg);
 	if(returnVal == 1) return 1;
    else returnVal = 0;
    
   	/////// ------- ////////
	// FINAL NAVIGATION
    returnVal = doNavigation(3, object_order[0], acNavigation, scanResponse[0]);
	if(returnVal == 1) return 1;
    else returnVal = 0;
    
 	/////// ------- ////////
    // PLACE
    
    returnVal = doPlace();
    if(returnVal == 1) return 1;
    else returnVal = 0;
	
	
	
    
    return 0;
}

