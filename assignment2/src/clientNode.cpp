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

    // 1st waypoint
    float degree_theta_z = 0.00;
    goal.x = 8.4;     
    goal.y = 0.0;     
    goal.z = 0.00;     
    degree_theta_z = 0.00; 
    goal.theta_z = degreesToRadians(degree_theta_z);
    acNavigation.sendGoal(goal, NULL, NULL, &feedbackNavigation);
    
    //waiting for result from server
    bool finished_before_timeout = acNavigation.waitForResult(ros::Duration(60.0));
 
	//print result
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
	
	//object_order[0]
    switch(object_order[0])
    {
    	case 1:
    		goal.x = 8.15;     
    		goal.y = -2.1;     
    		goal.z = 0.00;     
    		degree_theta_z = -110.00; 
    		goal.theta_z = degreesToRadians(degree_theta_z);
    		break;
    	case 2:
    		goal.x = 8.40;     
    		goal.y = -4.2;     
    		goal.z = 0.00;     
    		degree_theta_z = 90.00; 
    		goal.theta_z = degreesToRadians(degree_theta_z);
    		acNavigation.sendGoal(goal, NULL, NULL, &feedbackNavigation); 
    		finished_before_timeout = acNavigation.waitForResult(ros::Duration(60.0));
 
			//print result
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
   			 
    		goal.x = 7.50;     
    		goal.y = -4.00;     
    		goal.z = 0.00;     
    		degree_theta_z = 70.00; 
    		goal.theta_z = degreesToRadians(degree_theta_z);
    		break;
    	case 3:
    		goal.x = 7.20;     
    		goal.y = -2.1;     
    		goal.z = 0.00;     
    		degree_theta_z = -50.00; 
    		goal.theta_z = degreesToRadians(degree_theta_z);
    		break;
    	default:
    		ROS_ERROR("Error on selecting object ordering");
    		break;
    }
 	
 	//tiago reaches the first object position in front of table
    acNavigation.sendGoal(goal, NULL, NULL, &feedbackNavigation); 
    
    finished_before_timeout = acNavigation.waitForResult(ros::Duration(60.0));
 
	//print result
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
    
    assignment2::ArmGoal armGoal;
    actionlib::SimpleActionClient<assignment2::ArmAction> acManipulation("manipulationNode", true);

    if(detection_client.call(detection_srv)){
        ROS_INFO("Detection done, tag id returned in base_footprint reference frame");
        armGoal.request = 1; // PICK action is called
        armGoal.detections = detection_srv.response.detections;
        
        
        acManipulation.sendGoal(armGoal, NULL, NULL, &feedbackManipulation);
    }

    
    
    return 0;
}


