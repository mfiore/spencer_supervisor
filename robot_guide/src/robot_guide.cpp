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
#include <supervision_msgs/MoveToPoseAction.h>
#include <supervision_msgs/ApproachAction.h>

//messages
#include <geometry_msgs/PoseStamped.h>
#include <supervision_msgs/SupervisionStatus.h>
#include <situation_assessment_msgs/FactList.h>

//services
//#include <spencer_control_msgs/SetMaxVelocity.h>
#include <supervision_msgs/EmptyRequest.h>

//boost
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 
#include <boost/thread/condition_variable.hpp>


//self includes
#include <robot_guide/spencer_map.h>
#include <robot_guide/guide_pomdp.h>
#include <robot_guide/control_speed_pomdp.h>
#include <robot_guide/check_status.h>

#include <tf/transform_datatypes.h> 
#include <tf/transform_broadcaster.h>

//other
#include <string>
#include <vector>
#include <algorithm>    // std::max

#include <math.h>


using namespace std;

//useful typedefs for actions
typedef actionlib::SimpleActionServer<supervision_msgs::GuideGroupAction> GuideServer;
typedef actionlib::SimpleActionServer<supervision_msgs::MoveToAction> MoveToServer;
typedef actionlib::SimpleActionServer<supervision_msgs::MoveToPoseAction> MoveToPoseServer;
typedef actionlib::SimpleActionServer<supervision_msgs::ApproachAction> ApproachServer;

typedef actionlib::SimpleActionClient<supervision_msgs::GuideGroupAction> GuideClient;
typedef actionlib::SimpleActionClient<supervision_msgs::MoveToAction> MoveToClient;
typedef actionlib::SimpleActionClient<supervision_msgs::MoveToPoseAction> MoveToPoseClient;
typedef actionlib::SimpleActionClient<supervision_msgs::ApproachAction> ApproachClient;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//parameters
string robot_name;

double min_speed;
double max_speed;
double starting_speed;
double angular_velocity;
double actual_speed;

//pomdp observations and variables
boost::mutex mutex_observations;
boost::mutex mutex_has_observed_group;
boost::condition_variable condition_has_observed_group;

string delta_distance;
string group_distance;
string orientation;
string group_is_moving;
string highest_density;
string in_slow_area;

string timer="ok"; //not used at the moment

bool has_observed_group=false;
string group_to_observe="";

GuidePomdp* guide_pomdp;
ControlSpeedPomdp* control_speed_pomdp;

//location of the robot
string robot_location;

//ros services
ros::ServiceClient control_speed_client;
ros::ServiceClient switch_map_client;

//publishers
ros::Publisher status_pub;

//useful objects of the package
SpencerMap* spencer_map; //used to get the centroid of the sub maps
CheckStatus* check_status; //gets information about robot components

//control variables
bool stop_supervision=false;

bool simulation_mode; //if true doesn't connect to or call move_base

map<string,geometry_msgs::Pose> fake_map;
map<string,tf::Transform>> node_transforms;

bool hasMoveBaseError(MoveBaseClient* move_base_client) {
	if (!simulation_mode) {
	return move_base_client->getState()==actionlib::SimpleClientGoalState::ABORTED || 
				move_base_client->getState()==actionlib::SimpleClientGoalState::LOST ||
				move_base_client->getState()==actionlib::SimpleClientGoalState::PREEMPTED;
			}
			else return false;
}

string getCommonErrorString(MoveBaseClient* move_base_client, CheckStatus* check_status) {

	if (!ros::ok()) {
		return "node shutdown";
	}
	if (check_status->isBumperPressed()) {
		return "Bumper pressed or Emergency Stop";
	}
	else if (hasMoveBaseError(move_base_client)) {
		return "Move Base Error";
	}
	else if (check_status->isBatteryLow()) {
		return "Battery level is low";
	}
	else  {
		return "Other Error";
	}

}


bool switchMap(string node1, string node2) {
	annotated_mapping::SwitchMap srv;
	srv.request.name_1=node1;
	srv.request.name_2=node2;
	switch_map_client.call(srv);
	
	bool success=srv.response.success;

	if (success) {
		static tf::TransformBroadcaster br;
	  
	  	br.sendTransform(tf::StampedTransform(node_transforms[node1+"_"+node2], ros::Time::now(), "occ_map", "map"));
	}
}



void sendMoveBaseGoal(geometry_msgs::Pose pose,MoveBaseClient* move_base_client) {
	if (!simulation_mode) {
		ROS_INFO("Sending move base goal");

		move_base_msgs::MoveBaseGoal move_base_goal;
		move_base_goal.target_pose.header.frame_id = "map";
		move_base_goal.target_pose.header.stamp = ros::Time::now();
		move_base_goal.target_pose.pose=pose;

		move_base_client->sendGoal(move_base_goal);
		ROS_INFO("goal sent %f %f %f %f %f %f",move_base_goal.target_pose.pose.position.x, 
			move_base_goal.target_pose.pose.position.y, move_base_goal.target_pose.pose.orientation.x ,
			 move_base_goal.target_pose.pose.orientation.y , move_base_goal.target_pose.pose.orientation.z ,
			  move_base_goal.target_pose.pose.orientation.w);	
	}
}

void approach(const supervision_msgs::ApproachGoalConstPtr &goal, ApproachServer* approach_action_server,
	MoveBaseClient* move_base_client) {

}

void moveToPose(const supervision_msgs::MoveToPoseGoalConstPtr &goal, MoveToPoseServer* move_to_pose_action_server,
	MoveBaseClient* move_base_client) {

	supervision_msgs::MoveToPoseResult result;
	supervision_msgs::SupervisionStatus status_msg;

	ROS_INFO("Received request to move to %f %f",goal->pose.position.x,goal->pose.position.y);


	sendMoveBaseGoal(goal->pose,move_base_client);
	
	ros::Rate r(3);

	bool got_error, has_arrived,move_base_error,is_preempted;
	got_error=check_status->isBatteryLow()||check_status->isBumperPressed();
	has_arrived=false;
	move_base_error=false;
	is_preempted=false;

	while (!got_error && !has_arrived && !move_base_error && !move_to_pose_action_server) {
		has_arrived=move_base_client->getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
		got_error=check_status->isBatteryLow()||check_status->isBumperPressed() || !ros::ok();
		is_preempted=move_to_pose_action_server->isPreemptRequested();

		r.sleep();
	}

	if (has_arrived) {
		status_msg.status="Task Completed";
		status_msg.details="";

		ROS_INFO("Task completed");
		result.status="OK";
		status_pub.publish(status_msg);
		move_to_pose_action_server->setSucceeded(result);
	}
	else {
		ROS_INFO("Task Failed");

		status_msg.status="Task Failed";

		if (is_preempted) {
					status_msg.details="Preempted";
		}
		else {
			status_msg.details=getCommonErrorString(move_base_client,check_status);
		}
		status_pub.publish(status_msg);


		result.status="FAILURE";
		result.details=status_msg.details;
		ROS_INFO("Supervision is stopped");
		move_base_client->cancelGoal();

		if (move_to_pose_action_server->isPreemptRequested()) {
			move_to_pose_action_server->setPreempted(result);
		}
		else {
			move_to_pose_action_server->setAborted(result);
		} 
	}

}


void moveTo(const supervision_msgs::MoveToGoalConstPtr &goal,MoveToServer* move_to_action_server,MoveBaseClient* move_base_client) {
	//supervision will publish on a status topic as well as giving feedback for the actionn	
	supervision_msgs::MoveToFeedback feedback;
	supervision_msgs::MoveToResult result;
	supervision_msgs::SupervisionStatus status_msg; 

	if (goal->destination!="") {
		ROS_INFO("Received request to move to %s",goal->destination.c_str());
	}
	else if (goal->path.size()!=0) {
		ROS_INFO("Received request to move to %s",goal->path[goal->path.size()-1].c_str());
	}

	string destination=goal->destination;
	//get plan for destiantion
	vector<string> nodes; //list of nodes to traverse

	//get plan for destination (TODO)
	if (goal->path.size()!=0) {
		nodes=goal->path;
	}
	else if (destination!=""){

	}

	int current_node=0; 

	ros::Rate r(3); 

	bool task_completed=false;
	bool got_error=false;
	bool is_moving=false;

	got_error=check_status->isBatteryLow()||check_status->isBumperPressed();

	if (current_node<nodes.size()) {
		ROS_INFO("Switching to first map");
		if (switchMap(nodes[current_node],nodes[current_node+1])) {
			ROS_INFO("Starting to move");
			status_msg.details="Starting to move";
		}
		else {
			ROS_INFO("Error when switching map");
			got_error=true;
		}
	}

	while (!task_completed && !got_error && !move_to_action_server->isPreemptRequested()) {

		geometry_msgs::Pose goal_pose;
	
		double goal_x,goal_y;
		spencer_map->getMapCenter(nodes[current_node+1],&goal_x,&goal_y);
		goal_pose.position.x=goal_x;
		goal_pose.position.y=goal_y;		
		goal_pose.orientation.w=1.0;
		
		sendMoveBaseGoal(goal_pose,move_base_client);

		is_moving=true;

		status_msg.status="Moving to %s",nodes[current_node];
		status_msg.details="";
		status_pub.publish(status_msg);

		bool move_base_error=false;
		bool move_base_arrived=false;
		bool robot_arrived=false;
		while (!move_base_error && !move_base_arrived && !robot_arrived && !move_to_action_server->isPreemptRequested()) {
	
			move_base_error=hasMoveBaseError(move_base_client);
			move_base_arrived=move_base_client->getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
			robot_arrived=robot_location==nodes[current_node+1] && (current_node+1)!=nodes.size()-1;
			r.sleep();
		}

		if (robot_arrived || move_base_arrived && (current_node+1!=nodes.size()-1)) {
			move_base_client->cancelGoal();
			is_moving=false;

			current_node++;
			ROS_INFO("Reached node %s",nodes[current_node].c_str());

			ROS_INFO("Switching to new map");
			if(switchMap(nodes[current_node],nodes[current_node+1])){
				feedback.current_node=nodes[current_node];
				move_to_action_server->publishFeedback(feedback);	
			}
			else {
				ROS_INFO("Error when switching map");
				got_error=true;
			}

			ros::Duration(30).sleep();
		} 
		else if (got_error || move_base_error || move_to_action_server->isPreemptRequested() ) {
			break;
		}
		else if (move_base_arrived && (current_node+1)==nodes.size()-1) {
			task_completed=true;
		}
	}

	if (task_completed) {
		status_msg.status="Task Completed";
		status_msg.details="";

		ROS_INFO("Task completed");
		result.status="OK";
		status_pub.publish(status_msg);
		move_to_action_server->setSucceeded(result);
	}
	else {
		ROS_INFO("Task Failed");

		status_msg.status="Task Failed";

		if (move_to_action_server->isPreemptRequested()) {
					status_msg.details="Preempted";
		}
		else {
			status_msg.details=getCommonErrorString(move_base_client,check_status);
		}
		status_pub.publish(status_msg);


		result.status="FAILURE";
		result.details=status_msg.details;
		if (is_moving) {
			ROS_INFO("Supervision is stopped");
			move_base_client->cancelGoal();
			is_moving=false;
		}

		if (move_to_action_server->isPreemptRequested()) {
			move_to_action_server->setPreempted(result);
		}
		else {
			move_to_action_server->setAborted(result);
		} 
	}

}


//guide action callback
void guideGroup(const supervision_msgs::GuideGroupGoalConstPtr &goal,GuideServer* guide_action_server,
	MoveToClient* move_to_client, ApproachClient* approach_client) {
	//supervision will publish on a status topic as well as giving feedback for the actionn	
	supervision_msgs::GuideGroupFeedback feedback;
	supervision_msgs::GuideGroupResult result;
	supervision_msgs::SupervisionStatus status_msg; 

	ROS_INFO("Received request to guide group %s to %s",goal->group_id.c_str(),goal->destination.c_str());
	string destination=goal->destination;
	mutex_observations.lock();
	group_to_observe=goal->group_id;
	mutex_observations.unlock();

	ROS_INFO("Waiting for group observations");
	boost::unique_lock<boost::mutex> lock(mutex_has_observed_group);
	while (!has_observed_group) {
		condition_has_observed_group.wait(lock);
	}
	lock.unlock();

	vector<string> nodes; //list of nodes to traverse

	//get plan for destination (TODO)
	if (goal->path.size()) {
		nodes=goal->path;
	}

	int current_node=0; 

	ros::Rate r(3); 

	bool is_moving=false;
	bool task_completed=false;
	bool got_error=false;

	ROS_INFO("Starting POMDPs");
	//start pomdps
	mutex_observations.lock();
	string guide_action=guide_pomdp->update(timer,delta_distance,group_distance,orientation,group_is_moving);
	string speed_action=control_speed_pomdp->update(highest_density,in_slow_area);
	mutex_observations.unlock();

	//check if there is already an error
	got_error=check_status->isBatteryLow()||check_status->isBumperPressed();

	//the loop stops when the group abandons the task or we complete or we got an error or we are stopped from
	//the outside
	while (guide_action!="abandon" && task_completed==false && got_error==false && 
		stop_supervision==false && !guide_action_server->isPreemptRequested() && ros::ok()) {

		status_msg.status="Guiding group";
		if (guide_action=="continue") {
			//if the robot is not moving then get the next centroid and navigate towards it
			if (is_moving==false) { 
				supervision_msgs::MoveToGoal move_to_goal;
				move_to_goal.path=path;
				move_to_goal.destination=destination;
				move_to_client.sendGoal(move_to_goal);

				status_msg.details="Starting to move";

				// sendMoveBaseGoal(nodes[current_node+1],move_base_client);
				
				is_moving=true;

			}
			else {  //if it was already moving select a speed
				//control speed update

				mutex_observations.lock();
				speed_action=control_speed_pomdp->update(highest_density,in_slow_area);
				mutex_observations.unlock();

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
					move_base_client->cancelGoal();
				}
				ROS_INFO("Waiting for user");
				is_moving=false;
			}
		}
		got_error=hasMoveBaseError(move_base_client); 
		if (got_error){
			break;
		}

		bool move_base_completed=false;

		if (!simulation_mode)	{	
			move_base_completed=move_base_client->getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
		}

		//we switch map as soon as the robot arrives to the current destination, by stopping the robot and
		//calling the appropriate map service. We also publish a feedback in this situation.
		if ((robot_location==nodes[current_node+1] || move_base_completed) && (current_node+1)<nodes.size()-1) {
			is_moving==false;

			current_node++;
			if (!simulation_mode) {
				move_base_client->cancelGoal();
			}
			ROS_INFO("Reached node %s",nodes[current_node].c_str());
			ROS_INFO("Switching to new map");
			if (switchMap(nodes[current_node],nodes[current_node+1])) {
				feedback.current_node=nodes[current_node];
				guide_action_server->publishFeedback(feedback);
			}
			else {
				ROS_INFO("Error when switching map");
				got_error=true;
			}
	
		} 
		//The task is completed when we reach the centroid of the final destination.
		else if ((current_node+1)==nodes.size()-1 && move_base_completed) {
			task_completed=true;
		}
		if (!task_completed) {
			//update guide model
			mutex_observations.lock();
			guide_action=guide_pomdp->update(timer,delta_distance,group_distance,orientation,group_is_moving);
			mutex_observations.unlock();
		}

		status_pub.publish(status_msg);
		r.sleep();
	}

	mutex_has_observed_group.lock();
	has_observed_group=false;
	mutex_has_observed_group.unlock();

	if (task_completed) {
		status_msg.status="Task Completed";
		status_msg.details="";

		ROS_INFO("Task completed");
		result.status="OK";
		guide_action_server->setSucceeded(result);
	}
	else {
		status_msg.status="Task Failed";

		if (guide_action_server->isPreemptRequested()) {
			status_msg.details="Preempted";
		}
		else if (check_status->isBumperPressed()) {
			status_msg.details="Bumper pressed or Emergency Stop";
		}
		else if (hasMoveBaseError(move_base_client)) {
			status_msg.details="Move Base Error";
		}
		else if (check_status->isBatteryLow()) {
			status_msg.details="Battery level is low";
		}
		else if (guide_action=="abandon") {
			status_msg.details="Group lost";
		}
		else {
			status_msg.details="Other error";

		}
	

		ROS_INFO("Task Failed");
		result.status="FAILURE";
		result.details=status_msg.details;
		if (is_moving) {
			ROS_INFO("Supervision is stopped");
			if (!simulation_mode) {
				move_base_client->cancelGoal();
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


//Collect observations for the task
void agentFactCallback(const situation_assessment_msgs::FactList::ConstPtr& msg) {
	vector<situation_assessment_msgs::Fact> fact_list=msg->fact_list;

	int num_sides=0;
	int num_behind=0;

	bool found_group=false;

	boost::lock_guard<boost::mutex> guard(mutex_observations);

	vector<string> agents_in_group;
	BOOST_FOREACH(situation_assessment_msgs::Fact f, fact_list) {
		if (f.subject==group_to_observe && f.predicate[0]=="contains") {
			agents_in_group.push_back(f.value);
		}
		if (f.subject==group_to_observe && f.predicate[0]=="delta_distance" && f.predicate[1]==robot_name) {
			double delta_distance_num=boost::lexical_cast<double>(f.value);
			if (delta_distance_num<-1) {
				delta_distance="increasing";
			}
			else if (delta_distance_num>1) {
				delta_distance="decreasing";
			}
			else {
				delta_distance="stable";
			}
		}
		else if (f.subject==group_to_observe && f.predicate[0]=="distance" && f.predicate[1]==robot_name) {
			found_group=true;

			double distance_num=boost::lexical_cast<double>(f.value);
			if (distance_num<4) {
				group_distance="close";
			}
			else if (distance_num<10) {
				group_distance="far";
			}
			else group_distance="outOfRange";

			if (group_to_observe==agents_in_group[0]) {
				if (distance_num<1.5) {
					num_sides++;
					in_slow_area="false";
				}
				else if (distance_num<3) {
					num_behind++;
					in_slow_area="false";
				}
				else {
					in_slow_area="true";
				}
			}
		}
		else if (std::find(agents_in_group.begin(),agents_in_group.end(),f.subject)!=agents_in_group.end() &&
				 f.predicate[0]=="distance" && f.predicate[1]==robot_name) {
			double distance_num=boost::lexical_cast<double>(f.value);
			if (distance_num<1.5) {
				num_sides++;
				in_slow_area="false";
			}
			else if (distance_num<3) {
				num_behind++;
				in_slow_area="false";
			}
			else {
				in_slow_area="true";
			}
		}
		else if (f.subject==group_to_observe && f.predicate[0]=="isMoving") {
			if (f.value=="0") {
				group_is_moving="notMoving";
			}
			else {
				group_is_moving="moving";
			}
		}
		else if (f.subject==robot_name && f.predicate[0]=="isInArea") {
			robot_location=f.value;
		}


	}
	orientation="towardRobot";
	if (num_sides>num_behind) {
		highest_density="sides";
	}
	else {
		highest_density="behind";
	}

	boost::lock_guard<boost::mutex> lock(mutex_has_observed_group);
	if (found_group) {
			has_observed_group=true;
			condition_has_observed_group.notify_one();
	}
}

//stops the supervision system
bool stopSupervision(supervision_msgs::EmptyRequest::Request &req, supervision_msgs::EmptyRequest::Response &res) {
	stop_supervision=true;
	res.result="OK";
	return true;
}

//restarts the supervision system
bool restartSupervision(supervision_msgs::EmptyRequest::Request &req, supervision_msgs::EmptyRequest::Response &res) {
	stop_supervision=false;
	res.result="OK";
	return true;
}

int main(int argc, char **argv) {

	ros::init(argc,argv,"robot_guide");
	ros::NodeHandle n;

	ROS_INFO("Starting robot guide");

	string doc_path;
	string doc_name;

	//get all useful parameters
	n.getParam("/robot/name",robot_name);
	n.getParam("/supervision/doc_path",doc_path);
	n.getParam("/supervision/doc_name",doc_name);
	n.getParam("/supervision/angular_velocity",angular_velocity);
	n.getParam("/supervision/starting_speed",starting_speed);
	n.getParam("/supervision/min_speed",min_speed);
	n.getParam("/supervision/max_speed",max_speed);
	n.getParam("supervision/simulation_mode",simulation_mode);

	ROS_INFO("Parameters are:");
	ROS_INFO("robot name %s",robot_name.c_str());
	ROS_INFO("map documents path %s",doc_path.c_str());
	ROS_INFO("map documents base name %s",doc_name.c_str());
	ROS_INFO("angular velocity %f",angular_velocity);
	ROS_INFO("starting_speed %f",starting_speed);
	ROS_INFO("min_speed %f",min_speed);
	ROS_INFO("max_speed %f",max_speed);
	ROS_INFO("simulation_mode %d",simulation_mode);

	//create package objects
	guide_pomdp=new GuidePomdp(n);
	control_speed_pomdp=new ControlSpeedPomdp(n);
	//collect map informations, creating in the process areas for the agents
	spencer_map=new SpencerMap(n,doc_path,doc_name);
	if (!spencer_map->calculateMapInfos()) return 0;


	check_status=new CheckStatus(n);

		//connect to the action and servers
		ROS_INFO("Waiting for move_base");
		MoveBaseClient move_base_client("move_base",true);
	if (!simulation_mode) {

		move_base_client.waitForServer();
	}
	ROS_INFO("connected to move_base");


	ros::Rate r(10);

	ros::Subscriber agent_sub = n.subscribe("situation_assessment/agent_fact_list", 1000, agentFactCallback);
	ROS_INFO("Waiting for agent fact list to be published");
	while (agent_sub.getNumPublishers()==0 && ros::ok()) {
		r.sleep();
	}

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

	ROS_INFO("Connecting for switch map client");
	switch_map_client=n.serviceClient<annotated_mapping::SwitchMap>("switch_map");
	switch_map_client.waitForExistence();
	ROS_INFO("Connected");

	geometry_msgs::Pose pose_0,pose_1,pose_2,pose_3;
	pose_0.position.x=0;
	pose_0.position.y=0;
	pose_0.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,1.6);

	pose_1.position.x=2;
	pose_1.position.y=4.14;
	pose_1.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,1.6);


	pose_2.position.x=2.31;
	pose_2.position.y=10;	
	pose_2.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,1.6);


	pose_3.position.x=-0.92;
	pose_3.position.y=12;
	pose_3.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,4.7);


	fake_map["0"]=pose_0;
	fake_map["1"]=pose_1;
	fake_map["2"]=pose_2;
	fake_map["3"]=pose_2;

	// tf::Transform transform;
	// transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
	// tf::Quaternion q;
	// q.setRPY(0, 0, msg->theta);
	// transform.setRotation(q);


	if (!ros::ok()) {
		ROS_INFO("Shutdown request");
		return 0;
	}
	//advertise servers, topics and actions
	ros::ServiceServer stop_superivision_server=n.advertiseService("supervision/stop",stopSupervision);
	ros::ServiceServer restart_superivision_server=n.advertiseService("supervision/restart",restartSupervision);

	ROS_INFO("Started services to stop and restart supervisor");

	status_pub=n.advertise<supervision_msgs::SupervisionStatus>("supervision/status",1000);



	MoveToServer move_to_action_server(n,"supervision/move_to",
		boost::bind(&moveTo,_1,&move_to_action_server,&move_base_client),false);
	move_to_action_server.start();
	ROS_INFO("Started action server MoveTo");
 
 	MoveToPoseServer move_to_pose_action_server(n,"supervision/move_to_pose",
 		boost::bind(&moveToPose,_1,&move_to_pose_action_server,&move_base_client),false);
	move_to_pose_action_server.start();
	ROS_INFO("Started action server MoveToPose"); 

	ApproachServer approach_action_server(n,"supervision/approach",
		boost::bind(&approach,_1,&approach_action_server,&move_base_client),false);
	approach_action_server.start();
	ROS_INFO("Started action server Approach");

	MoveToClient move_to_client("supervision/move_to",true);
	ApproachClient approach_client("supervision/approach",true);
	MoveToPoseClient move_to_pose_client("supervision/move_to_pose",true);



	GuideServer guide_action_server(n,"supervision/guide_group",
		boost::bind(&guideGroup,_1,&guide_action_server,&move_to_client, &approach_client),false);
	guide_action_server.start();
	ROS_INFO("Started action server GuideGroup");

	ROS_INFO("Ready");

	ros::spin();
	return 0;
}