#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment2/PoseAction.h>
//#inlcude <assignment2/ArmAction.h>
#include <tiago_iaslab_simulation/Objs.h>
#include "utils.h"


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

    //////// ------ ////////
    
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
    goal.x = 8.4;     
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

    //////// ------ ////////

    ros::ServiceClient detection_client = nh.serviceClient<assignment2::Detection>("/object_detection");
    
    assignment2::Detection detection_srv;
    detection_srv.request.ready = true;
    detection_srv.request.requested_id = object_order[0];

    std::vector<std::pair<geometry_msgs::Pose, int>> pairsOfPoses;
    
    if(detection_client.call(detection_srv)){
        for(int i = 0; i < detection_srv.response.poses.size(); i++)
        {
            geometry_msgs::Pose currentPose = detection_srv.response.poses[i];
            int ids = detection_srv.response.poses_ids[i];
            float sizes = detection_srv.response.poses_sizes[i];

        	ROS_INFO("POSITION %d for ID %d", i, ids);
        	ROS_INFO("X: %f", currentPose.position.x);
        	ROS_INFO("Y: %f", currentPose.position.y);
        	ROS_INFO("Z: %f", currentPose.position.z);
            ROS_INFO("ORIENTATION %d for ID %d", i, ids);
            ROS_INFO("X: %f", currentPose.orientation.x);
        	ROS_INFO("Y: %f", currentPose.orientation.y);
        	ROS_INFO("Z: %f", currentPose.orientation.z);
            ROS_INFO("W: %f", currentPose.orientation.w);

            std::pair<geometry_msgs::Pose, int> poseIdPair(currentPose, ids);
            pairsOfPoses.push_back(poseIdPair);
        }
    }
    else{
    	ROS_ERROR("Failed to call service to detect object's tag on table");
    }


    // ros::ServiceClient manipulationClient = nh.serviceClient<assignment2::Arm>("/manipulate_object");

    
    
    return 0;
}


