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
#include <spencer_control_msgs/SetMaxVelocity.h>
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
double speed_adaptation_time_;
double first_stop_time_;

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
double test_mode; //number of seconds to sleep at the start

//encapsulate move base errors
bool hasMoveBaseError(MoveBaseClient* move_base_client) {
	if (!simulation_mode) {
	return move_base_client->getState()==actionlib::SimpleClientGoalState::ABORTED || 
				move_base_client->getState()==actionlib::SimpleClientGoalState::LOST ||
				move_base_client->getState()==actionlib::SimpleClientGoalState::PREEMPTED;
			}
			else return false;
}

bool hasMoveToError(MoveToClient* move_to_client) {
	if (!simulation_mode) {
	return move_to_client->getState()==actionlib::SimpleClientGoalState::ABORTED || 
				move_to_client->getState()==actionlib::SimpleClientGoalState::LOST ||
				move_to_client->getState()==actionlib::SimpleClientGoalState::PREEMPTED;
			}
			else return false;
}

bool hasSystemError(CheckStatus* check_status, bool is_moving, MoveToClient* move_to_client) {
	bool result=false;
	if (is_moving) {
		result=hasMoveToError(move_to_client);
	}
	return result || check_status->isBatteryLow() || check_status->isStopped();
}

bool isPaused(CheckStatus* check_status) {
	return check_status->isPaused() || check_status->isBumperPressed();
}


// Called every time feedback is received for the goal
void moveToFeedbackCb(const supervision_msgs::MoveToFeedbackConstPtr& feedback)
{
	if (feedback->current_node!="") {
  		ROS_INFO("ROBOT_GUIDE Move to has arrived in %s", feedback->current_node.c_str());
  	}
  	else {
  		ROS_INFO("ROBOT_GUIDE Move to has arrived in %f %f",feedback->current_pose.position.x,
  												feedback->current_pose.position.y);
  	}
  current_node_in_plan++;
}


vector<string> getAgentsInGroup() {
	vector<string> result;
	situation_assessment_msgs::QueryDatabase entity_query;

	ROS_INFO("ROBOT_GUIDE ROBOT_GUIDE getting agents in group");

	entity_query.request.query.model=robot_name;
	entity_query.request.query.subject="";
	entity_query.request.query.predicate.push_back("isInArea");

	if (!database_client.call(entity_query)) {
		ROS_ERROR("Couldn't contact database");
		return result;
	}
	vector<string> entities_in_area;
	ROS_INFO("ROBOT_GUIDE ROBOT_GUIDE getting entities in area");
	for (int i=0; i<entity_query.response.result.size();i++) {
		string entity_name=entity_query.response.result[i].subject;
		vector<string> areas=entity_query.response.result[i].value;
		if (std::find(areas.begin(),areas.end(),robot_name)!=areas.end()) {
			entities_in_area.push_back(entity_name);
		}
	}
	ROS_INFO("entities in areas size %ld",entities_in_area.size());
	vector<string> humans_in_area;
	ROS_INFO("ROBOT_GUIDE ROBOT_GUIDE getting humans in area");
	for (int i=0; i<entities_in_area.size();i++) {
		situation_assessment_msgs::QueryDatabase human_query;
		human_query.request.query.subject=entities_in_area[i];
		human_query.request.query.predicate.push_back("type");
		if (!database_client.call(human_query)) {
			ROS_ERROR("Couldn't connect to database");
			return result;
		}
		if (human_query.response.result.size()==0) {
			ROS_ERROR("ROBOT_GUIDE error! no human returned!");
			return result;
		}
		else if (human_query.response.result[0].value.size()<2) {
			ROS_ERROR("ROBOT_GUIDE error! human class incomplete");
			return result;
		}
		else if (human_query.response.result[0].value[1]=="human") {
			humans_in_area.push_back(entities_in_area[i]);
		}
		ROS_INFO("query response size %ld",human_query.response.result.size());
	}
	result=humans_in_area;
	return result;

}

void switchDrivingDirection(bool backward, ros::ServiceClient* set_driving_direction_client) {
	//when the robot guides people, it will move backward, so that they can see the screen
	ROS_INFO("Setting driving direction to %d",backward);
	spencer_nav_msgs::SetDrivingDirection set_driving_direction_srv;
	set_driving_direction_srv.request.backward=backward;
	if (!simulation_mode && use_driving_direction) {
		if (set_driving_direction_client->call(set_driving_direction_srv)){
			ROS_INFO("Done");
		
			
	}
		else {
			ROS_WARN("Couldn't switch driving direction");
		}
	}
	situation_assessment_msgs::SwitchOrientation srv_switch_orientation;
	srv_switch_orientation.request.backward=backward;
			if(!switch_orientation_client.call(srv_switch_orientation)){
			  ROS_ERROR("ROBOT_GUIDE couldn't call switch orientation");
			}
}

void switchSpeed(double newSpeed, ros::ServiceClient* control_speed_client) {
	if (!simulation_mode) {
	spencer_control_msgs::SetMaxVelocity srv;
	srv.request.max_linear_velocity=newSpeed;
	srv.request.max_angular_velocity=angular_velocity;
	control_speed_client->call(srv);
	}
	ROS_INFO("ROBOT_GUIDE switching speed to %f",newSpeed);
}


//guide action callback. Goal can contains a destination (the robot will plan to find the symbolic path), a symbolic path
//or coordinates. The priority for goals follow this order (so if we want to give the robot the path, we shouldn't set its
//destination)
void guideGroup(const supervision_msgs::GuideGroupGoalConstPtr &goal,GuideServer* guide_action_server,
	MoveToClient* move_to_client, ObservationManager* observation_manager,
	GuidePomdp* guide_pomdp, ControlSpeedPomdp* control_speed_pomdp, 
	ros::ServiceClient* set_driving_direction_client, ros::ServiceClient* calculate_path_client) {
	//supervision will publish on a status topic as well as giving feedback for the actionn	
	supervision_msgs::GuideGroupFeedback feedback;
	supervision_msgs::GuideGroupResult result;
	supervision_msgs::SupervisionStatus status_msg; 


	SupervisionTimer wait_timer(time_to_wait); //timer used when waiting users (before aborting)
	SupervisionTimer accelerate_timer(speed_adaptation_time_);
	SupervisionTimer decelerate_timer(speed_adaptation_time_);
	SupervisionTimer stop_timer(first_stop_time_);

	//boost::thread timer_thread; //its thread
	//boost::thread accelerate_timer_thread;
	//boost::thread decelerate_timer_thread;
	//boost::thread stop_timer_thread;


	//get mission information
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
		ROS_INFO("ROBOT_GUIDE Received request to guide group %s to %s",group_id.c_str(),destination.c_str());
	}
	else {
		ROS_INFO("ROBOT_GUIDE Received request to guide group %s to %f %f",group_id.c_str(),last_pose.position.x,
			last_pose.position.y);
	}
	//when the robot guides people, it will move backward, so that they can see the screen
	switchDrivingDirection(true,set_driving_direction_client);

	if (test_mode>0) {
		ROS_INFO("ROBOT_GUIDE sleeping %f seconds for test mode",test_mode);
		ros::Duration(test_mode).sleep();
	}
	//wait until the group is seen (note that simple mode is transparent to this procedure)
	// observation_manager->setObservedGroup(group_id);
	// observation_manager->waitForGroup();
	vector<string> agents_in_group=getAgentsInGroup();
	if (agents_in_group.size()==0) {
		ROS_WARN("ROBOT_GUIDE No Agents behind the robot. Aborting");
		result.status="FAILED";
		result.details="no agents present";
		guide_action_server->setAborted();
		return;
	}
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



	ROS_INFO("ROBOT_GUIDE Starting POMDPs");
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

	//if robot is paused wait
	while (isPaused(check_status) && !hasSystemError(check_status,is_moving,move_to_client)) {
		status_msg.status="supervision is paused";
		status_pub.publish(status_msg);
		r.sleep();
	}

	boost::thread first_time_thread(boost::bind(&SupervisionTimer::start,&stop_timer));

	//check if there is already an error
	got_error=check_status->isBatteryLow() || check_status->isBumperPressed() || check_status->isStopped();
    is_preempted=guide_action_server->isPreemptRequested();


	switchSpeed(starting_speed,&control_speed_client);	//switch to starting speed
    actual_speed=starting_speed;	
//the loop stops when the group abandons the task or we complete or we got an error or we are stopped from
	//the outside
	while (guide_action!="abandon" && !task_completed && !hasSystemError(check_status,is_moving,move_to_client) &&
	 !guide_action_server->isPreemptRequested()  && ros::ok()) {

		status_msg.status="RUNNING";
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

				if (destination!="") {
					status_msg.details="guiding group to "+destination;		
				}
				else {
					status_msg.details="guiding group to "+boost::lexical_cast<string>(last_pose.position.x)+" "+
					boost::lexical_cast<string>(last_pose.position.y);
				}
				is_moving=true;
			}
			else if (use_control_speed) {  //if it was already moving select a speed
				//control speed update

				string speed_action=control_speed_pomdp->update(
					observation_manager->getHighestDensity(),
					observation_manager->getInSlowArea());


				if (speed_action=="accelerate"){
					if (!accelerate_timer.isRunning()) {
						boost::thread accelerate_timer_thread(boost::bind(&SupervisionTimer::start,&accelerate_timer));
						 double new_speed=min(actual_speed+0.1,max_speed);

   	  					if (new_speed!=actual_speed) {
						 	ROS_INFO("ROBOT_GUIDE Robot is accelerating");
						 	actual_speed=new_speed;
						 	switchSpeed(new_speed,&control_speed_client);
							status_msg.details="Accelerating";

						}
					}
				}

				else if (speed_action=="decelerate") {
				  //	  ROS_INFO("action is decelerate");
				  //  ROS_INFO("timer is running %d",decelerate_timer.isRunning());
					if (!decelerate_timer.isRunning()) {
					  //	  ROS_INFO("thread is running");
						boost::thread decelerate_timer_thread(boost::bind(&SupervisionTimer::start,&decelerate_timer));
						double new_speed=max(actual_speed-0.1,min_speed);
					//	ROS_INFO("new speed %f actual speed %f",new_speed,actual_speed);
						if (new_speed!=actual_speed) {
	     					ROS_INFO("ROBOT_GUIDE is decelerating");
								switchSpeed(new_speed,&control_speed_client);
							actual_speed=new_speed;
							status_msg.details="decelerating";
						}
					}
				}
				else if (speed_action=="continue") {
					if (destination!="") {
						status_msg.details="guiding group to "+destination;		
					}
					else {
						status_msg.details="guiding group to "+boost::lexical_cast<string>(last_pose.position.x)+" "
						+boost::lexical_cast<string>(last_pose.position.y);
					}			
				}
			}
			else {
				if (destination!="") {
					status_msg.details="guiding group to "+destination;		
				}
				else {
					status_msg.details="guiding group to "+boost::lexical_cast<string>(last_pose.position.x)+" "+
					boost::lexical_cast<string>(last_pose.position.y);
				}	
			}
		}
		else if (guide_action=="wait") { //if the pomdp selects a wait stop move base and start the timer
			if (is_moving==true && stop_timer.isElapsed()) { //we also check the timer to disallow the robot to stop in the first seconds
				status_msg.details="waiting for group";

				//start timer
				boost::thread t(boost::bind(&SupervisionTimer::start,&wait_timer));

				if (!simulation_mode) {
					move_to_client->cancelGoal();
			}
				ROS_INFO("ROBOT_GUIDE Waiting for user");
				is_moving=false;
			}
		}
		if (isPaused(check_status)) {
			ROS_INFO("Robot is paused");
			while (isPaused(check_status) && !hasSystemError(check_status,is_moving,move_to_client)) {
				is_moving=false;
				status_msg.status="supervision is paused";
				status_msg.details="";
				status_pub.publish(status_msg);
				r.sleep();
			}
		}
		if (is_moving) {
			if (check_status->isPlannerBlocked()) {
				is_moving=false;
			}
			while (check_status->isPlannerBlocked() && !hasSystemError(check_status,is_moving,move_to_client)) {
				r.sleep();
			}
			task_completed=move_to_client->getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
		}

		if (!hasSystemError(check_status,is_moving,move_to_client) && !guide_action_server->isPreemptRequested()) {
			status_pub.publish(status_msg);
			r.sleep();

			//update observations, starting with the timer
			string s_timer;
			if (wait_timer.isElapsed()) {
				s_timer="expired";
				ROS_INFO("ROBOT_GUIDE Timer expired");
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

				ROS_INFO("ROBOT_GUIDE Abandoning task");
			}
		}
	}
	wait_timer.stop();
	
	accelerate_timer.stop();
	decelerate_timer.stop();
	stop_timer.stop();

	
	//at the end of the task reset the driving direction to forward
	if (!ros::ok()) return;
	switchDrivingDirection(false,set_driving_direction_client);
	switchSpeed(max_speed,&control_speed_client);
	actual_speed=max_speed;

	//publish final status
	if (task_completed) {
		status_msg.status="COMPLETED";
		status_msg.details="";

		ROS_INFO("ROBOT_GUIDE Task completed");
		result.status="COMPLETED";
		result.details="";
		guide_action_server->setSucceeded(result);

		status_pub.publish(status_msg);

	}
	else {
		status_msg.status="FAILED";
		
		if (guide_action=="abandon") {
		    status_msg.details="Group Lost";
		  }
		
		  else if (guide_action_server->isPreemptRequested()) {
			ROS_INFO("ROBOT_GUIDE Guide preempted");
			status_msg.details="Preempted";
		}
		else if (move_to_client->getState()==actionlib::SimpleClientGoalState::ABORTED || 	
			 	 move_to_client->getState()==actionlib::SimpleClientGoalState::LOST ||
			     move_to_client->getState()==actionlib::SimpleClientGoalState::PREEMPTED) {

			status_msg.details="Navigation Error";
		ROS_INFO("ROBOT_GUIDE Navigation Error");
		}
		else  {
			status_msg.details=check_status->getCommonErrorString();
		}
	

		ROS_INFO("ROBOT_GUIDE Guide Task Failed");
		result.status="FAILED";
		result.details=status_msg.details;
		if (is_moving) {
			ROS_INFO("ROBOT_GUIDE Supervision is stopped");
			if (!simulation_mode) {
				move_to_client->cancelGoal();
			}
			is_moving=false;
		}
		status_pub.publish(status_msg);

		if (guide_action_server->isPreemptRequested()) {
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

	ROS_INFO("ROBOT_GUIDE Setting driving direction to forward");
	spencer_nav_msgs::SetDrivingDirection set_driving_direction_srv;
	set_driving_direction_srv.request.backward=false;
	if (!simulation_mode) {
		if (set_driving_direction_client->call(set_driving_direction_srv)){
			ROS_INFO("ROBOT_GUIDE Done");
		}
		else {
			ROS_WARN("Couldn't switch driving direction");
		}
	}

}



int main(int argc, char **argv) {

	ros::init(argc,argv,"robot_guide");
	ros::NodeHandle n;

	ROS_INFO("ROBOT_GUIDE Starting robot guide");

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
	n.getParam("supervision/test_mode",test_mode);
	n.getParam("supervision/speed_adaptation_time",speed_adaptation_time_);
	n.getParam("supervision/first_stop_time",first_stop_time_);

	ROS_INFO("ROBOT_GUIDE Parameters are:");
	ROS_INFO("ROBOT_GUIDE robot name %s",robot_name.c_str());
	ROS_INFO("ROBOT_GUIDE angular velocity %f",angular_velocity);
	ROS_INFO("ROBOT_GUIDE starting_speed %f",starting_speed);
	ROS_INFO("ROBOT_GUIDE min_speed %f",min_speed);
	ROS_INFO("ROBOT_GUIDE max_speed %f",max_speed);
	ROS_INFO("ROBOT_GUIDE simulation_mode %d",simulation_mode);
	ROS_INFO("ROBOT_GUIDE mode %s",observations_mode.c_str());
	ROS_INFO("ROBOT_GUIDE wait time is %f",time_to_wait);
	ROS_INFO("ROBOT_GUIDE Use Control speed is %d",use_control_speed);
	ROS_INFO("ROBOT_GUIDE Use driving direction is %d",use_driving_direction);
	ROS_INFO("ROBOT_GUIDE Test mode of %f",test_mode);
	ROS_INFO("ROBOT_GUIDE Speed adaptation time is of %f",speed_adaptation_time_);
	ROS_INFO("ROBOT_GUIDE First stop time is  %f",first_stop_time_);

	//create package objects
	GuidePomdp guide_pomdp(n);
	ControlSpeedPomdp control_speed_pomdp(n);

	ObservationManager observation_manager(n,robot_name,observations_mode);
	check_status=new CheckStatus(n);

	ros::Rate r(10);


	ROS_INFO("ROBOT_GUIDE Connecting to control speed");
	control_speed_client=n.serviceClient<spencer_control_msgs::SetMaxVelocity>("/spencer/control/set_max_velocity",true);
	if (use_control_speed) {
		ROS_INFO("ROBOT_GUIDE Starting speed is %f",starting_speed);
		 actual_speed=starting_speed;

        if (!simulation_mode) {
			control_speed_client.waitForExistence();
			ROS_INFO("Connected");
			switchSpeed(starting_speed,&control_speed_client);
		}
	}

	if (!ros::ok()) {
		ROS_INFO("ROBOT_GUIDE Shutdown request");
		return 0;
	}

	ROS_INFO("ROBOT_GUIDE Connecting to set driving direction service\n");
	ros::ServiceClient set_driving_direction_client=n.serviceClient<spencer_nav_msgs::SetDrivingDirection>("/spencer/nav/set_driving_direction",true);
	if (!simulation_mode && use_driving_direction) {
		set_driving_direction_client.waitForExistence();
		ROS_INFO("ROBOT_GUIDE Connected\n");
	}

	ROS_INFO("ROBOT_GUIDE Connecting to calculate path service\n");
	ros::ServiceClient calculate_path_client=n.serviceClient<supervision_msgs::CalculatePath>("/supervision/calculate_path",true);
	calculate_path_client.waitForExistence();
	ROS_INFO("ROBOT_GUIDE Connected\n");	

	ROS_INFO("ROBOT_GUIDE Connecting to switch orientation service\n");
	switch_orientation_client=n.serviceClient<situation_assessment_msgs::SwitchOrientation>("/situation_assessment/switch_orientation",true);
	switch_orientation_client.waitForExistence();
	ROS_INFO("ROBOT_GUIDE Connected\n");

	//		situation_assessment_msgs::SwitchOrientation srv_switch_orientation;
	//		srv_switch_orientation.request.backward=false;
	//if(!switch_orientation_client.call(srv_switch_orientation)){
	//		ROS_ERROR("ROBOT_GUIDE couldn't call switch orientation");
	//}
	switchDrivingDirection(false,&set_driving_direction_client);
	


	ROS_INFO("ROBOT_GUIDE Connecting to query database service\n");
	database_client=n.serviceClient<situation_assessment_msgs::QueryDatabase>("/situation_assessment/query_database",true);
	database_client.waitForExistence();
	ROS_INFO("ROBOT_GUIDE Connected\n");



	ROS_INFO("ROBOT_GUIDE Waiting for move_base");
	MoveBaseClient move_base_client("move_base",true);
	if (!simulation_mode) {
		move_base_client.waitForServer();
	}
	ROS_INFO("ROBOT_GUIDE connected to move_base");

	ROS_INFO("ROBOT_GUIDE Started services to stop and restart supervisor");

	status_pub=n.advertise<supervision_msgs::SupervisionStatus>("supervision/robot_guide/status",1000);

	ApproachServer approach_action_server(n,"supervision/approach",
		boost::bind(&approach,_1,&approach_action_server,
			&move_base_client,&set_driving_direction_client),false);
	approach_action_server.start();
	ROS_INFO("ROBOT_GUIDE Started action server Approach");

	MoveToClient move_to_client("supervision/move_to",true);
	ApproachClient approach_client("supervision/approach",true);

	GuideServer guide_action_server(n,"supervision/guide_group",
		boost::bind(&guideGroup,_1,&guide_action_server,&move_to_client, &observation_manager,
			&guide_pomdp,&control_speed_pomdp, &set_driving_direction_client,&calculate_path_client),false);
	guide_action_server.start();

	ROS_INFO("ROBOT_GUIDE Started action server GuideGroup");

	supervision_msgs::SupervisionStatus status_msg; 

	status_msg.status="IDLE";
	status_pub.publish(status_msg);
	ROS_INFO("ROBOT_GUIDE Ready");

	ros::spin();
	return 0;
}
