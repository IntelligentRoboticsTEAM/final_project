#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment2/PoseAction.h>
#include <tiago_iaslab_simulation/Objs.h>
#include "utils.h"
#include "assignment2/Detection.h"


/**
 * @brief Callback function to handle feedback from the PoseAction server.
 * @param feedback The feedback received from the server.
 */
void feedbackCallback(const assignment2::PoseFeedbackConstPtr& feedback) {
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
    
    actionlib::SimpleActionClient<assignment2::PoseAction> ac("poseRevisited", true);
    ROS_INFO("Waiting for action server to start.");
    
    if (!ac.waitForServer(ros::Duration(5.0))) { // Wait for 5 seconds
        ROS_ERROR("Action server not available");
        return 1;
    }
    
    ROS_INFO("Action server started.");

    assignment2::PoseGoal goal;
    float degree_theta_z = 0.00;
 	
 	//Waypoint Goal
    goal.x = 8.3;     
    goal.y = 0.0;     
    goal.z = 0.00;     
    degree_theta_z = 0.00; 
    goal.theta_z = degreesToRadians(degree_theta_z);
    ac.sendGoal(goal, NULL, NULL, &feedbackCallback);
    
    //waiting for result from server
    bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
 
	//print result
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            const auto& result = *ac.getResult();        
        }
    } else {
        ROS_INFO("Action did not finish before the timeout.");
        ac.cancelGoal();
        ROS_INFO("Goal has been cancelled");
    }
	
	
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
    		goal.x = 8.30;     
    		goal.y = -4.2;     
    		goal.z = 0.00;     
    		degree_theta_z = 90.00; 
    		goal.theta_z = degreesToRadians(degree_theta_z);
    		ac.sendGoal(goal, NULL, NULL, &feedbackCallback); 
    		finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
 
			//print result
    		if (finished_before_timeout) {
       		 actionlib::SimpleClientGoalState state = ac.getState();
       		 ROS_INFO("Action finished: %s", state.toString().c_str());
        		if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            		const auto& result = *ac.getResult();        
        		}
   			 } else {
        		ROS_INFO("Action did not finish before the timeout.");
       			 ac.cancelGoal();
       			 ROS_INFO("Goal has been cancelled");
   			 }
   			 
    		goal.x = 7.50;     
    		goal.y = -4.00;     
    		goal.z = 0.00;     
    		degree_theta_z = 90.00; 
    		goal.theta_z = degreesToRadians(degree_theta_z);
    		break;
    	case 3:
    		goal.x = 7.20;     
    		goal.y = -2.1;     
    		goal.z = 0.00;     
    		degree_theta_z = -70.00; 
    		goal.theta_z = degreesToRadians(degree_theta_z);
    		break;
    	default:
    		ROS_ERROR("Error on selecting object ordering");
    		break;
    }
 	
    ac.sendGoal(goal, NULL, NULL, &feedbackCallback); //tiago reaches the first object position in front of table
    
    finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
 
	//print result
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            const auto& result = *ac.getResult();        
        }
    } else {
        ROS_INFO("Action did not finish before the timeout.");
        ac.cancelGoal();
        ROS_INFO("Goal has been cancelled");
    }
    
	//ros::NodeHandle nh;
    /*ros::ServiceClient detection_client = nh.serviceClient<assignment2::Detection>("/object_detection");
    
    assignment2::Detection detection_srv;
    detection_srv.request.ready = true;
    
    if(client.call(srv)){
    	ROS_INFO("The service returned control to client");
    }
    else{
    	ROS_ERROR("Failed to call service to get object sequence");
    }
    */

    return 0;
}


