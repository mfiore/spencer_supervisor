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
#include <supervision_msgs/CalculatePath.h>
#include <spencer_nav_msgs/SetDrivingDirection.h>
#include <situation_assessment_msgs/SwitchOrientation.h>
#include <situation_assessment_msgs/QueryDatabase.h>


//self includes
#include <robot_guide/guide_pomdp.h>
#include <robot_guide/control_speed_pomdp.h>
#include <robot_guide/observation_manager.h>
#include <spencer_status/check_status.h>
#include <robot_guide/supervision_timer.h>


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

//these are used for speed adaptation
double min_speed; 
double max_speed;
double starting_speed;
double angular_velocity;
double actual_speed;

//max time the robot will wait for users before abandoning
double time_to_wait;

//explained in observation_manager.h
string observations_mode;

//control which additional modules are on or off
bool use_driving_direction;
bool use_control_speed;


//location of the robot
int current_node_in_plan;

//ros services
ros::ServiceClient control_speed_client;
ros::ServiceClient switch_orientation_client;
ros::ServiceClient database_client;

//publishers
ros::Publisher status_pub;

CheckStatus* check_status; //gets information about robot components

bool simulation_mode; //if true doesn't connect to or call move_base


//encapsulate move base errors
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
	if (feedback->current_node!="") {
  		ROS_INFO("Move to has arrived in %s", feedback->current_node.c_str());
  	}
  	else {
  		ROS_INFO("Move to has arrived in %f %f",feedback->current_pose.position.x,
  												feedback->current_pose.position.y);
  	}
  current_node_in_plan++;
}


vector<string> getAgentsInGroup() {
	vector<string> result;
	situation_assessment_msgs::QueryDatabase srv;

	ROS_INFO("ROBOT_GUIDE getting agents in group");

	srv.request.query.model=robot_name;
	srv.request.query.subject="";
	srv.request.query.predicate.push_back("isInArea");

	if (!database_client.call(srv)) {
		ROS_ERROR("Couldn't contact database");
		return result;
	}
	vector<string> entities_in_area;
	ROS_INFO("ROBOT_GUIDE getting entities in area");
	for (int i=0; i<srv.response.result.size();i++) {
		string entity_name=srv.response.result[i].subject;
		vector<string> areas=srv.response.result[i].value;
		if (std::find(areas.begin(),areas.end(),robot_name)!=areas.end()) {
			entities_in_area.push_back(entity_name);
		}
	}
	vector<string> humans_in_area;
	ROS_INFO("ROBOT_GUIDE getting humans in area");
	for (int i=0; i<entities_in_area.size();i++) {
		srv.request.query.subject=entities_in_area[i];
		srv.request.query.predicate[0]="type";
		if (!database_client.call(srv)) {
			ROS_ERROR("Couldn't connect to database");
			return result;
		}
		if (srv.response.result[0].value[1]=="HUMAN") {
			humans_in_area.push_back(entities_in_area[i]);
		}
	}
	ROS_INFO("ROBOT_GUIDE returning");
	result=humans_in_area;
	return result;


}


//guide action callback. Goal can contains a destination (the robot will plan to find the symbolic path), a symbolic path
//or coordinates. The priority for goals follow this order (so if we want to give the robot the path, we shouldn't set its
//destination)
void guideGroup(const supervision_msgs::GuideGroupGoalConstPtr &goal,GuideServer* guide_action_server,
	MoveToClient* move_to_client, ApproachClient* approach_client, ObservationManager* observation_manager,
	GuidePomdp* guide_pomdp, ControlSpeedPomdp* control_speed_pomdp, 
	ros::ServiceClient* set_driving_direction_client, ros::ServiceClient* calculate_path_client) {
	//supervision will publish on a status topic as well as giving feedback for the actionn	
	supervision_msgs::GuideGroupFeedback feedback;
	supervision_msgs::GuideGroupResult result;
	supervision_msgs::SupervisionStatus status_msg; 


	SupervisionTimer wait_timer(time_to_wait);
	boost::thread timer_thread;

	string group_id=goal->group_id;
	string destination;
	geometry_msgs::Pose last_pose;

	if (group_id=="") {
		ROS_WARN("Group id missing from request");
		result.status="FAILED";
		result.details="group id is missing";
		guide_action_server->setAborted(result);
		return;
	}

	if (goal->destination!="") {
		destination=goal->destination;
	}
	else if (goal->path.size()>0) {
		destination=goal->path[goal->path.size()-1];
	} 
	else if (goal->coordinates.size()>0) {
		last_pose=goal->coordinates[goal->coordinates.size()-1];
	}
	else {
		ROS_WARN("No destination or path given");
		result.status="FAILED";
		result.details="no destination or path given";
		guide_action_server->setAborted();
		return;
	}

	if (destination!="") {
		ROS_INFO("Received request to guide group %s to %s",group_id.c_str(),destination.c_str());
	}
	else {
		ROS_INFO("Received request to guide group %s to %f %f",group_id.c_str(),last_pose.position.x,
			last_pose.position.y);
	}

	//wait until the group is seen (note that simple mode is transparent to this procedure)
	// observation_manager->setObservedGroup(group_id);
	// observation_manager->waitForGroup();
	vector<string> agents_in_group=getAgentsInGroup();
	observation_manager->setAgentsInGroup(agents_in_group);

	vector<string> nodes; //list of nodes to traverse
	vector<geometry_msgs::Pose> poses; //list of nodes to traverse
	if (goal->path.size()!=0) {
		nodes=goal->path;
	}
	if (goal->coordinates.size()>0) {
		poses=goal->coordinates;
	}
	else {
		// supervision_msgs::CalculatePath path_request;
		// path_request.request.source=observation_manager->getRobotLocation();
		// path_request.request.dest=destination;
		// if (calculate_path_client->call(path_request)) {
		// 	nodes=path_request.response.path;
		// }
		// else {
		// 	ROS_ERROR("Failed to calculate path");
		// 	result.status="FAILED";
		// 	result.details="group id is missing";
		// 	guide_action_server->setAborted(result);
		// 	return;
		// }
	}
	current_node_in_plan=0;  //we start from node number 0 in the plan


	ros::Rate r(3); 

	//control variables 
	bool is_moving=false;
	bool task_completed=false;
	bool got_error=false;
	bool is_preempted=false;

	//when the robot guides people, it will move backward, so that they can see the screen
	ROS_INFO("Setting driving direction to backward");
	spencer_nav_msgs::SetDrivingDirection set_driving_direction_srv;
	set_driving_direction_srv.request.backward=true;
	if (!simulation_mode && use_driving_direction) {
		if (set_driving_direction_client->call(set_driving_direction_srv)){
			ROS_INFO("Done");
		}
		else {
			ROS_WARN("Couldn't switch driving direction");
		}
	}
	situation_assessment_msgs::SwitchOrientation srv_switch_orientation;
	srv_switch_orientation.request.backward=true;
	if(!switch_orientation_client.call(srv_switch_orientation)){
		ROS_ERROR("ROBOT_GUIDE couldn't call switch orientation");
	}


	ROS_INFO("Starting POMDPs");
	//start pomdps
		string guide_action=guide_pomdp->update(
			"ok",
			observation_manager->getDeltaDistance(),
			observation_manager->getGroupDistance(),
			observation_manager->getOrientation(),
			observation_manager->getGroupIsMoving());

		string speed_action=control_speed_pomdp->update(
			observation_manager->getHighestDensity(),
			observation_manager->getInSlowArea());

		ROS_INFO("%s a %s b %s c %s",observation_manager->getDeltaDistance().c_str(),observation_manager->getGroupDistance().c_str(),
			observation_manager->getOrientation().c_str(),observation_manager->getGroupIsMoving().c_str());

	//check if there is already an error
	got_error=check_status->isBatteryLow() || check_status->isBumperPressed() || check_status->isStopped();
    is_preempted=guide_action_server->isPreemptRequested();
	//the loop stops when the group abandons the task or we complete or we got an error or we are stopped from
	//the outside

	int n_tries=0;
	while (guide_action!="abandon" && !task_completed && !got_error && !is_preempted
		 && ros::ok()) {

		// if (n_tries<10) {
		// 	n_tries++;
		// 	guide_action="continue";
		// }

		status_msg.status="Guiding group";
		if (guide_action=="continue") {
			wait_timer.stop();

			//if the robot is not moving then get the next centroid and navigate towards it
			if (is_moving==false) { 
				supervision_msgs::MoveToGoal move_to_goal;
				
				if (destination=="") {
					vector<string> actual_path;
					vector<geometry_msgs::Pose> actual_poses;
					//we take the planner's path from current node to end. We do this if not the MoveTo action
					//would replan everytime we stop the semantic path.
					if (nodes.size()>0) {
						vector<string>::const_iterator first = nodes.begin() + current_node_in_plan;
						vector<string>::const_iterator last = nodes.end();
						vector<string> actual_path(first, last);
					}
					if (poses.size()>0) {
						vector<geometry_msgs::Pose>::const_iterator first = poses.begin() + current_node_in_plan;
						vector<geometry_msgs::Pose>::const_iterator last = poses.end();
						actual_poses.insert(actual_poses.begin(),first, last);
					}
					move_to_goal.path=actual_path;
					move_to_goal.coordinates=actual_poses;
				}

				move_to_goal.destination=destination;
								move_to_client->sendGoal(
									move_to_goal,
									MoveToClient::SimpleDoneCallback(),
									MoveToClient::SimpleActiveCallback(),
					&moveToFeedbackCb);

				status_msg.details="Starting to move";		
				is_moving=true;
			}
			else if (use_control_speed) {  //if it was already moving select a speed
				//control speed update

				string speed_action=control_speed_pomdp->update(
					observation_manager->getHighestDensity(),
					observation_manager->getInSlowArea());

				if (speed_action=="accelerate"){

					 double new_speed=min(actual_speed+0.1,max_speed);
					// spencer_control_msgs::SetMaxVelocity srv;
					// srv.request.max_linear_velocity=actual_speed;
					// srv.request.max_angular_velocity=angular_velocity;
					// control_speed_client.call(srv);
   	  				if (new_speed!=actual_speed) {
					 	ROS_INFO("Switching speed to %f",new_speed);
					 	actual_speed=new_speed;
						status_msg.details="Accelerating";

					}
				}

				else if (speed_action=="decelerate") {

					 double new_speed=max(actual_speed-0.1,min_speed);
					// spencer_control_msgs::SetMaxVelocity srv;
					// srv.request.max_linear_velocity=actual_speed;
					// srv.request.max_angular_velocity=angular_velocity;
					// control_speed_client.call(srv);
					 if (new_speed!=actual_speed) {
						ROS_INFO("Switching speed to %f",new_speed);
						actual_speed=new_speed;
						status_msg.details="Decelerating";
					}
				}
				else if (speed_action=="continue") {
					status_msg.details="Moving";
				}
			}
		}
		else if (guide_action=="wait") { //if the pomdp selects a wait stop move base and start the timer

			if (is_moving==true) {
				status_msg.details="Waiting for users";

				boost::thread t(boost::bind(&SupervisionTimer::start,&wait_timer));

				if (!simulation_mode) {
					move_to_client->cancelGoal();
			}
				ROS_INFO("Waiting for user");
				is_moving=false;
			}
		}
		//check for errors 
		got_error=check_status->isBatteryLow() || check_status->isBumperPressed() || check_status->isStopped();
		if (is_moving) {
                  move_to_client->getState()==actionlib::SimpleClientGoalState::ABORTED || 	
			 	  move_to_client->getState()==actionlib::SimpleClientGoalState::LOST ||
			      move_to_client->getState()==actionlib::SimpleClientGoalState::PREEMPTED;
	    }		    
	    is_preempted=guide_action_server->isPreemptRequested();

	    //if we're not in simulation check if move to has arrived to destination
		if (!simulation_mode)	{	
			task_completed=move_to_client->getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
		}

		status_pub.publish(status_msg);
		r.sleep();

		//update observations, starting with the timer
		string s_timer;
		if (wait_timer.isElapsed()) {
			s_timer="ok";
			ROS_INFO("Timer expired");
		}
		else {
			s_timer="ok";
		}
		guide_action=guide_pomdp->update(
			s_timer,
			observation_manager->getDeltaDistance(),
			observation_manager->getGroupDistance(),
			observation_manager->getOrientation(),
			observation_manager->getGroupIsMoving());
		if (guide_action=="abandon") {
			ROS_INFO("Abandoning task");
		}
	}
	
	//at the end of the task reset the driving direction to forward
	if (!ros::ok()) return;
	ROS_INFO("Setting driving direction to forward");
	set_driving_direction_srv.request.backward=false;
	if (!simulation_mode && use_driving_direction) {
		if (set_driving_direction_client->call(set_driving_direction_srv)){
			ROS_INFO("Done");
		}
		else {
			ROS_WARN("Couldn't switch driving direction");
		}
	}
	srv_switch_orientation.request.backward=false;
	if(!switch_orientation_client.call(srv_switch_orientation)){
		ROS_ERROR("ROBOT_GUIDE couldn't call switch orientation");
	}

	//publish final status
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
			ROS_INFO("Guide preempted");
			status_msg.details="Preempted";
		}
		else if (move_to_client->getState()==actionlib::SimpleClientGoalState::ABORTED || 	
			 	 move_to_client->getState()==actionlib::SimpleClientGoalState::LOST ||
			     move_to_client->getState()==actionlib::SimpleClientGoalState::PREEMPTED) {
			status_msg.details="Navigation Error";
		ROS_INFO("Navigation Error");
		}
		else  {
			status_msg.details=check_status->getCommonErrorString();
		}
	

		ROS_INFO("Guide Task Failed");
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

//approaches a group. Still needs to be interfaced with Omar software so at the moment this is a stub
void approach(const supervision_msgs::ApproachGoalConstPtr &goal, ApproachServer* approach_action_server,
	MoveBaseClient* move_base_client, ros::ServiceClient* set_driving_direction_client) {

	ROS_INFO("Setting driving direction to forward");
	spencer_nav_msgs::SetDrivingDirection set_driving_direction_srv;
	set_driving_direction_srv.request.backward=false;
	if (!simulation_mode) {
		if (set_driving_direction_client->call(set_driving_direction_srv)){
			ROS_INFO("Done");
		}
		else {
			ROS_WARN("Couldn't switch driving direction");
		}
	}

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
	n.getParam("supervision/use_driving_direction",use_driving_direction);
	n.getParam("supervision/use_control_speed",use_control_speed);
	n.getParam("supervision/observations_mode",observations_mode);
	n.getParam("supervision/wait_time",time_to_wait);

	ROS_INFO("Parameters are:");
	ROS_INFO("robot name %s",robot_name.c_str());
	ROS_INFO("angular velocity %f",angular_velocity);
	ROS_INFO("starting_speed %f",starting_speed);
	ROS_INFO("min_speed %f",min_speed);
	ROS_INFO("max_speed %f",max_speed);
	ROS_INFO("simulation_mode %d",simulation_mode);
	ROS_INFO("mode %s",observations_mode.c_str());
	ROS_INFO("wait time is %f",time_to_wait);
	ROS_INFO("Use Control speed is %d",use_control_speed);
	ROS_INFO("Use driving direction is %d",use_driving_direction);

	//create package objects
	GuidePomdp guide_pomdp(n);
	ControlSpeedPomdp control_speed_pomdp(n);

	ObservationManager observation_manager(n,robot_name,observations_mode);
	check_status=new CheckStatus(n);

	ros::Rate r(10);


	// ROS_INFO<<"Connecting to control speed\n";
	// // control_speed_client=n.serviceClient<spencer_control_msgs::SetMaxVelocity>("/spencer/control/set_max_velocity",true);
	// if (use_control_speed) {
	// 	control_speed_client.waitForExistence();
	// 	ROS_INFO<<"Connected\n";

	// 	// set the starting speed of the robot
	// 	spencer_control_msgs::SetMaxVelocity srv;
	// 	srv.request.max_linear_velocity=starting_speed;
	// 	srv.request.max_angular_velocity=angular_velocity;
	// 	control_speed_client.call(srv);
	// 	cout<<"Starting speed is "<<starting_speed<<"\n";
	// 	 actual_speed=starting_speed;
	// }
	
	if (!ros::ok()) {
		ROS_INFO("Shutdown request");
		return 0;
	}

	ROS_INFO("Connecting to set driving direction service\n");
	ros::ServiceClient set_driving_direction_client=n.serviceClient<spencer_nav_msgs::SetDrivingDirection>("/spencer/navigation/set_driving_direction",true);
	if (!simulation_mode && use_driving_direction) {
		set_driving_direction_client.waitForExistence();
		ROS_INFO("Connected\n");
	}

	ROS_INFO("Connecting to calculate path service\n");
	ros::ServiceClient calculate_path_client=n.serviceClient<supervision_msgs::CalculatePath>("/supervision/calculate_path",true);
	calculate_path_client.waitForExistence();
	ROS_INFO("Connected\n");	

	ROS_INFO("Connecting to switch orientation service\n");
	switch_orientation_client=n.serviceClient<situation_assessment_msgs::SwitchOrientation>("/situation_assessment/switch_orientation",true);
	switch_orientation_client.waitForExistence();
	ROS_INFO("Connected\n");

	situation_assessment_msgs::SwitchOrientation srv_switch_orientation;
	srv_switch_orientation.request.backward=true;
	if(!switch_orientation_client.call(srv_switch_orientation)){
		ROS_ERROR("ROBOT_GUIDE couldn't call switch orientation");
	}


	ROS_INFO("Connecting to query database service\n");
	database_client=n.serviceClient<situation_assessment_msgs::QueryDatabase>("/situation_assessment/query_database",true);
	database_client.waitForExistence();
	ROS_INFO("Connected\n");



	ROS_INFO("Waiting for move_base");
	MoveBaseClient move_base_client("move_base",true);
	if (!simulation_mode) {
		move_base_client.waitForServer();
	}
	ROS_INFO("connected to move_base");

	ROS_INFO("Started services to stop and restart supervisor");

	status_pub=n.advertise<supervision_msgs::SupervisionStatus>("supervision/status",1000);

	ApproachServer approach_action_server(n,"supervision/approach",
		boost::bind(&approach,_1,&approach_action_server,
			&move_base_client,&set_driving_direction_client),false);
	approach_action_server.start();
	ROS_INFO("Started action server Approach");

	MoveToClient move_to_client("supervision/move_to",true);
	ApproachClient approach_client("supervision/approach",true);

	GuideServer guide_action_server(n,"supervision/guide_group",
		boost::bind(&guideGroup,_1,&guide_action_server,&move_to_client, &approach_client, &observation_manager,
			&guide_pomdp,&control_speed_pomdp, &set_driving_direction_client,&calculate_path_client),false);
	guide_action_server.start();

	ROS_INFO("Started action server GuideGroup");

	ROS_INFO("Ready");

	ros::spin();
	return 0;
}