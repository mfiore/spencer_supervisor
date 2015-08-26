/**
robot_guide.cpp

author Michelangelo Fiore.

Main file for the spencer supervisor, which guides groups of passengers to a destination.
*/

#include <ros/ros.h>

//actions
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <supervision_msgs/GuideGroupAction.h>
#include <supervision_msgs/MoveToAction.h>
#include <supervision_msgs/ApproachAction.h>

//messages
#include <supervision_msgs/SupervisionStatus.h>
#include <situation_assessment_msgs/FactList.h>

//services
//#include <spencer_control_msgs/SetMaxVelocity.h>
#include <supervision_msgs/EmptyRequest.h>




#include <robot_guide/guide_pomdp.h>
#include <robot_guide/control_speed_pomdp.h>
#include <robot_guide/observation_manager.h>

#include <spencer_status/check_status.h>


//other
#include <string>
#include <vector>
#include <algorithm>    // std::max

#include <math.h>


using namespace std;

//useful typedefs for actions
typedef actionlib::SimpleActionServer<supervision_msgs::GuideGroupAction> GuideServer;
typedef actionlib::SimpleActionServer<supervision_msgs::ApproachAction> ApproachServer;

typedef actionlib::SimpleActionClient<supervision_msgs::MoveToAction> MoveToClient;
typedef actionlib::SimpleActionClient<supervision_msgs::ApproachAction> ApproachClient;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//parameters
string robot_name;

double min_speed;
double max_speed;
double starting_speed;
double angular_velocity;
double actual_speed;



//location of the robot
int current_node_in_plan;


//ros services
ros::ServiceClient control_speed_client;
ros::ServiceClient switch_map_client;

//publishers
ros::Publisher status_pub;

CheckStatus* check_status; //gets information about robot components

bool simulation_mode; //if true doesn't connect to or call move_base


bool hasMoveBaseError(MoveBaseClient* move_base_client) {
	if (!simulation_mode) {
	return move_base_client->getState()==actionlib::SimpleClientGoalState::ABORTED || 
				move_base_client->getState()==actionlib::SimpleClientGoalState::LOST ||
				move_base_client->getState()==actionlib::SimpleClientGoalState::PREEMPTED;
			}
			else return false;
}




// Called every time feedback is received for the goal
void moveToFeedbackCb(const supervision_msgs::MoveToFeedbackConstPtr& feedback)
{
  ROS_INFO("Move to as arrived in %s", feedback->current_node.c_str());
  current_node_in_plan++;
}


//guide action callback
void guideGroup(const supervision_msgs::GuideGroupGoalConstPtr &goal,GuideServer* guide_action_server,
	MoveToClient* move_to_client, ApproachClient* approach_client, ObservationManager* observation_manager,
	GuidePomdp* guide_pomdp, ControlSpeedPomdp* control_speed_pomdp) {
	//supervision will publish on a status topic as well as giving feedback for the actionn	
	supervision_msgs::GuideGroupFeedback feedback;
	supervision_msgs::GuideGroupResult result;
	supervision_msgs::SupervisionStatus status_msg; 

	string destination=goal->destination;
	string group_id=goal->group_id;

	ROS_INFO("Received request to guide group %s to %s",group_id.c_str(),destination.c_str());

	observation_manager->setObservedGroup(group_id);
	observation_manager->waitForGroup();

	vector<string> nodes; //list of nodes to traverse
	//get plan for destination (TODO)
	if (goal->path.size()) {
		nodes=goal->path;
	}
	current_node_in_plan=0;


	ros::Rate r(3); 

	bool is_moving=false;
	bool task_completed=false;
	bool got_error=false;
	bool is_preempted=false;

	ROS_INFO("Starting POMDPs");
	//start pomdps
	string guide_action=guide_pomdp->update(
		observation_manager->getTimer(),
		observation_manager->getDeltaDistance(),
		observation_manager->getGroupDistance(),
		observation_manager->getOrientation(),
		observation_manager->getGroupIsMoving());

	string speed_action=control_speed_pomdp->update(
		observation_manager->getHighestDensity(),
		observation_manager->getInSlowArea());

	//check if there is already an error
	got_error=check_status->isBatteryLow() || check_status->isBumperPressed() || check_status->isStopped();
    is_preempted=guide_action_server->isPreemptRequested();
	//the loop stops when the group abandons the task or we complete or we got an error or we are stopped from
	//the outside
	while (guide_action!="abandon" && !task_completed && !got_error && !is_preempted
		 && ros::ok()) {

		status_msg.status="Guiding group";
		if (guide_action=="continue") {
			//if the robot is not moving then get the next centroid and navigate towards it
			if (is_moving==false) { 
				supervision_msgs::MoveToGoal move_to_goal;
				
				//we take the planner's path from current node to end. We do this if not the MoveTo action
				//would replan everytime we stop the semantic path.
				vector<string>::const_iterator first = nodes.begin() + current_node_in_plan;
				vector<string>::const_iterator last = nodes.end();
				vector<string> actual_path(first, last);

				move_to_goal.path=actual_path;
				move_to_goal.destination=destination;
								move_to_client->sendGoal(
									move_to_goal,
									MoveToClient::SimpleDoneCallback(),
									MoveToClient::SimpleActiveCallback(),
					&moveToFeedbackCb);

				status_msg.details="Starting to move";		
				is_moving=true;
			}
			else {  //if it was already moving select a speed
				//control speed update
				string speed_action=control_speed_pomdp->update(
					observation_manager->getHighestDensity(),
					observation_manager->getInSlowArea());

				if (speed_action=="accelerate"){
					status_msg.details="Accelerating";

					 double new_speed=min(actual_speed+0.1,max_speed);
					// spencer_control_msgs::SetMaxVelocity srv;
					// srv.request.max_linear_velocity=actual_speed;
					// srv.request.max_angular_velocity=angular_velocity;
					// control_speed_client.call(srv);
   	  				if (new_speed!=actual_speed) {
					 	ROS_INFO("Switching speed to %f",new_speed);
					 	actual_speed=new_speed;
					}
				}
				else if (speed_action=="decelerate") {
					status_msg.details="Decelerating";

					 double new_speed=max(actual_speed-0.1,min_speed);
					// spencer_control_msgs::SetMaxVelocity srv;
					// srv.request.max_linear_velocity=actual_speed;
					// srv.request.max_angular_velocity=angular_velocity;
					// control_speed_client.call(srv);
					 if (new_speed!=actual_speed) {
						ROS_INFO("Switching speed to %f",new_speed);
						actual_speed=new_speed;
					}
				}
				else if (speed_action=="continue") {
					status_msg.details="Moving";
				}
			}
		}
		else if (guide_action=="wait") { //if the pomdp selects a wait stop move base
			if (is_moving==true) {
				status_msg.details="Waiting for users";

				if (!simulation_mode) {
					move_to_client->cancelGoal();
			}
				ROS_INFO("Waiting for user");
				is_moving=false;
			}
		}
		got_error=check_status->isBatteryLow() || check_status->isBumperPressed() || check_status->isStopped() ||
                  move_to_client->getState()==actionlib::SimpleClientGoalState::ABORTED || 	
			 	  move_to_client->getState()==actionlib::SimpleClientGoalState::LOST ||
			      move_to_client->getState()==actionlib::SimpleClientGoalState::PREEMPTED;
	    is_preempted=guide_action_server->isPreemptRequested();

		if (!simulation_mode)	{	
			task_completed=move_to_client->getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
		}

		status_pub.publish(status_msg);
		r.sleep();
		guide_action=guide_pomdp->update(
				observation_manager->getTimer(),
				observation_manager->getDeltaDistance(),
				observation_manager->getGroupDistance(),
				observation_manager->getOrientation(),
				observation_manager->getGroupIsMoving());
	}


	if (task_completed) {
		status_msg.status="Task Completed";
		status_msg.details="";

		ROS_INFO("Task completed");
		result.status="OK";
		guide_action_server->setSucceeded(result);
	}
	else {
		status_msg.status="Task Failed";

		if (is_preempted) {
			status_msg.details="Preempted";
		}
		else if (move_to_client->getState()==actionlib::SimpleClientGoalState::ABORTED || 	
			 	 move_to_client->getState()==actionlib::SimpleClientGoalState::LOST ||
			     move_to_client->getState()==actionlib::SimpleClientGoalState::PREEMPTED) {
			status_msg.details="Navigation Error";
		}
		else  {
			status_msg.details=check_status->getCommonErrorString();
		}
	

		ROS_INFO("Task Failed");
		result.status="FAILURE";
		result.details=status_msg.details;
		if (is_moving) {
			ROS_INFO("Supervision is stopped");
			if (!simulation_mode) {
				move_to_client->cancelGoal();
			}
			is_moving=false;
		}
		status_pub.publish(status_msg);

		if (is_preempted) {
			guide_action_server->setPreempted(result);
		}
		else {
			guide_action_server->setAborted(result);
		} 
	}
}



void approach(const supervision_msgs::ApproachGoalConstPtr &goal, ApproachServer* approach_action_server,
	MoveBaseClient* move_base_client) {

}



int main(int argc, char **argv) {

	ros::init(argc,argv,"robot_guide");
	ros::NodeHandle n;

	ROS_INFO("Starting robot guide");

	//get all useful parameters
	n.getParam("/robot/name",robot_name);

	n.getParam("/supervision/angular_velocity",angular_velocity);
	n.getParam("/supervision/starting_speed",starting_speed);
	n.getParam("/supervision/min_speed",min_speed);
	n.getParam("/supervision/max_speed",max_speed);
	n.getParam("supervision/simulation_mode",simulation_mode);

	ROS_INFO("Parameters are:");
	ROS_INFO("robot name %s",robot_name.c_str());
	ROS_INFO("angular velocity %f",angular_velocity);
	ROS_INFO("starting_speed %f",starting_speed);
	ROS_INFO("min_speed %f",min_speed);
	ROS_INFO("max_speed %f",max_speed);
	ROS_INFO("simulation_mode %d",simulation_mode);

	//create package objects
	GuidePomdp guide_pomdp(n);
	ControlSpeedPomdp control_speed_pomdp(n);

	ObservationManager observation_manager(n,robot_name);
	check_status=new CheckStatus(n);

	ros::Rate r(10);


	// ROS_INFO<<"Connecting to control speed\n";
	// control_speed_client=n.serviceClient<spencer_control_msgs::SetMaxVelocity>("/spencer/control/set_max_velocity",true);
	// control_speed_client.waitForExistence();
	// ROS_INFO<<"Connected\n";

	//set the starting speed of the robot
	// spencer_control_msgs::SetMaxVelocity srv;
	// srv.request.max_linear_velocity=starting_speed;
	// srv.request.max_angular_velocity=angular_velocity;
	// control_speed_client.call(srv);
	// cout<<"Starting speed is "<<starting_speed<<"\n";
	 actual_speed=starting_speed;

	
	if (!ros::ok()) {
		ROS_INFO("Shutdown request");
		return 0;
	}

	ROS_INFO("Waiting for move_base");
	MoveBaseClient move_base_client("move_base",true);
	if (!simulation_mode) {
		move_base_client.waitForServer();
	}
	ROS_INFO("connected to move_base");

	ROS_INFO("Started services to stop and restart supervisor");

	status_pub=n.advertise<supervision_msgs::SupervisionStatus>("supervision/status",1000);

	ApproachServer approach_action_server(n,"supervision/approach",
		boost::bind(&approach,_1,&approach_action_server,&move_base_client),false);
	approach_action_server.start();
	ROS_INFO("Started action server Approach");

	MoveToClient move_to_client("supervision/move_to",true);
	ApproachClient approach_client("supervision/approach",true);

	GuideServer guide_action_server(n,"supervision/guide_group",
		boost::bind(&guideGroup,_1,&guide_action_server,&move_to_client, &approach_client, &observation_manager,
			&guide_pomdp,&control_speed_pomdp),false);
	guide_action_server.start();

	ROS_INFO("Started action server GuideGroup");

	ROS_INFO("Ready");

	ros::spin();
	return 0;
}