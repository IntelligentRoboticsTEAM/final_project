#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment2/PoseAction.h>
#include <assignment2/ArmAction.h>
#include <tiago_iaslab_simulation/Objs.h>
#include "utils.h"


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

    //////// ------ ////////
    
    actionlib::SimpleActionClient<assignment2::PoseAction> acNavigation("poseRevisited", true);
    ROS_INFO("Waiting for action server to start.");
    
    if (!acNavigation.waitForServer(ros::Duration(5.0))) { // Wait for 5 seconds
        ROS_ERROR("Action server not available");
        return 1;
    }
    ROS_INFO("Action server started.");

    assignment2::PoseGoal goal;
    goal.id = object_order[0];
    acNavigation.sendGoal(goal, NULL, NULL, &feedbackNavigation); 
    bool finished_before_timeout = acNavigation.waitForResult(ros::Duration(60.0));
    
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = acNavigation.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            const auto& result = *acNavigation.getResult();        
        }
    } else {
        ROS_INFO("Action did not finish before the timeout.");
        acNavigation.cancelGoal();
        ROS_INFO("Goal has been cancelled");
    }

    //////// ------ ////////

    ros::ServiceClient detection_client = nh.serviceClient<assignment2::Detection>("/object_detection");
    
    assignment2::Detection detection_srv;
    detection_srv.request.ready = true;
    detection_srv.request.requested_id = object_order[0];
   	bool flag = false;
	assignment2::ArmGoal armGoal;
	std::vector<apriltag_ros::AprilTagDetection> detectionsObj;
	
    if(detection_client.call(detection_srv)){
        ROS_INFO("Detection done, tag id returned in base_footprint reference frame");
        
        armGoal.request = 1; // PICK action is called
        detectionsObj = detection_srv.response.detections;  
        armGoal.detections = detectionsObj;
      	flag = true;
      	ROS_INFO("detection_srv.detections.size = %d ", (int)detection_srv.response.detections.size());
        ROS_INFO("detection_srv.detections.size = %d ", (int)detectionsObj.size());
        ROS_INFO("detection_srv.detections.size = %d ", (int)armGoal.detections.size());
    }
    
    if(flag){
		for (int i = 0; i < armGoal.detections.size(); i++){
			ROS_INFO("detection_srv.detections[%d].id[0] = %d ", i, (int)detection_srv.response.detections[i].id[0]);
			ROS_INFO("detectionsObj[%d].id[0] = %d ", i, (int)detectionsObj[i].id[0]);
			ROS_INFO("armGoal.detections[%d].id[0] = %d ", i, (int)armGoal.detections[i].id[0]);
		}
		actionlib::SimpleActionClient<assignment2::ArmAction> acManipulation("manipulationNode", true);
		acManipulation.sendGoal(armGoal, NULL, NULL, &feedbackManipulation);
	} else {
		ROS_ERROR("Manipulation not working, ERROR IN CLIENT");
	}
    
    return 0;
}

