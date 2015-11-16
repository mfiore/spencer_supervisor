/**
	robot_navigation.cpp
	author: Michelangelo Fiore

	This class acts as supervision for robot navigation tasks.
*/


#include <ros/ros.h>
//actions
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

//msgs
#include <move_base_msgs/MoveBaseAction.h>
#include <supervision_msgs/MoveToAction.h>

#include <tf/transform_datatypes.h> 
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <supervision_msgs/SupervisionStatus.h>
#include <situation_assessment_msgs/FactList.h>
#include <supervision_msgs/CalculatePath.h>



#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 
#include <boost/lexical_cast.hpp>

//services
#include <annotated_mapping/SwitchMap.h>
#include <situation_assessment_msgs/QueryDatabase.h>
#include <situation_assessment_msgs/EmptyRequest.h>


//libraries
#include <spencer_status/check_status.h>

//some useful typedefs
typedef actionlib::SimpleActionServer<supervision_msgs::MoveToAction> MoveToServer;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//services
ros::ServiceClient switch_map_client;
ros::ServiceClient simple_database_client;
// ros::ServiceClient has_published_client;

//location of the robot
vector<string> robot_areas;
string robot_location;
string next_area;
bool reached_next_area_=false;

bool use_map_switching;

boost::mutex mutex_location;


//parameters
string robot_name;
bool simulation_mode;

map<string, geometry_msgs::Pose> fake_map; //can be useful when we don't want to read the ma pdocuments

CheckStatus *check_status; //contains the symbolic status of the robot's system (batteries, bumper, emergency switch..)

//publishers
ros::Publisher status_pub;

map<string,geometry_msgs::Point> node_centers_; //includes the centers of each symbolic node


geometry_msgs::Pose getPose(string name) {
	situation_assessment_msgs::QueryDatabase srv;

	situation_assessment_msgs::Fact fact;

	fact.model=robot_name;
	fact.subject=name;
	fact.predicate.push_back("pose");
	srv.request.query=fact;

	geometry_msgs::Pose pose;
	if (simple_database_client.call(srv)) {
		if (srv.response.result.size()==0) {
			ROS_ERROR("Couldn't fine pose for %s",name.c_str());
		}
		else {
			pose.position.x=boost::lexical_cast<double>(srv.response.result[0].value[0]);
			pose.position.y=boost::lexical_cast<double>(srv.response.result[0].value[1]);
			pose.position.z=boost::lexical_cast<double>(srv.response.result[0].value[2]);
			pose.orientation.x=boost::lexical_cast<double>(srv.response.result[0].value[3]);
			pose.orientation.y=boost::lexical_cast<double>(srv.response.result[0].value[4]);
			pose.orientation.z=boost::lexical_cast<double>(srv.response.result[0].value[5]);
			pose.orientation.w=boost::lexical_cast<double>(srv.response.result[0].value[6]);
		}
	}
	else {
		ROS_ERROR("Could not contact database");
	}
	return pose;
}

bool hasArea(string name) {
	situation_assessment_msgs::QueryDatabase srv;
	situation_assessment_msgs::Fact fact;

	fact.model=robot_name;
	fact.subject=name;
	fact.predicate.push_back("hasArea");
	srv.request.query=fact;;


	if (simple_database_client.call(srv)) {
		if (srv.response.result.size()>0) {
			return true;
		}
	}
	else {
		ROS_ERROR("Could not contact database");
	}
	return false;
}

string getEntityType(string name) {
	situation_assessment_msgs::QueryDatabase srv;
	situation_assessment_msgs::Fact fact;

	fact.model=robot_name;
	fact.subject=name;
	fact.predicate.push_back("type");
	srv.request.query=fact;

	if (simple_database_client.call(srv)) {
		if (srv.response.result.size()>0) {
			return srv.response.result[0].value[0];
		}
	}
	else {
		ROS_ERROR("Could not contact database");
	}
	return "";	
}

string getRobotLocation() {
	boost::lock_guard<boost::mutex> lock(mutex_location);
	// ROS_INFO("ROBOT_NAVIGATION Robot areas size %ld",robot_areas.size());
	for (int i=0; i<robot_areas.size();i++) {
		if (robot_areas[i]!="this") {

			situation_assessment_msgs::QueryDatabase srv;

			situation_assessment_msgs::Fact fact;

			fact.model=robot_name;
			fact.subject=robot_areas[i];
			fact.predicate.push_back("type");
			srv.request.query=fact;


			if (simple_database_client.call(srv)) {
				if (srv.response.result.size()>0) {
					// ROS_INFO("ROBOT_NAVIGATION %s area is type %s",robot_areas[i].c_str(),srv.response.result[0].value[0].c_str());
					if (srv.response.result[0].value[0]=="location") {
						ROS_INFO("ROBOT_NAVIGATION found robot location");
						return robot_areas[i];
					}
				}
				else {
					ROS_ERROR("Could not contact database");
				}
			}
		}
	}
	return "this";			
}

string getEntityLocation(string name) {
	vector<string> entity_areas;
	situation_assessment_msgs::QueryDatabase srv;

	situation_assessment_msgs::Fact fact;

	fact.model=robot_name;
	fact.subject=name;
	fact.predicate.push_back("isInArea");
	srv.request.query=fact;

	if (simple_database_client.call(srv)) {
		entity_areas=srv.response.result[0].value;

		for (int i=0; i<entity_areas.size();i++) {
			if (entity_areas[i]!="this") {
		 		fact.predicate.clear();

				fact.model=robot_name;
				fact.subject=entity_areas[i];
				fact.predicate.push_back("type");
				srv.request.query=fact;

				if (simple_database_client.call(srv)) {
					if (srv.response.result.size()>0) {
						if (srv.response.result[0].value[0]=="location") return entity_areas[i];
					}
				}
				else {
					ROS_ERROR("Could not contact database");
					break;
				}
			}
		}
	}
	else {
		ROS_ERROR("Could not contact database");
	}
	return "this";
}

//returns true if there is a move base error. 
bool hasMoveBaseError(MoveBaseClient* move_base_client) {
	if (!simulation_mode) {
	return move_base_client->getState()==actionlib::SimpleClientGoalState::ABORTED || 
				move_base_client->getState()==actionlib::SimpleClientGoalState::LOST ||
				move_base_client->getState()==actionlib::SimpleClientGoalState::PREEMPTED;
			}
			else return false;
}

//incapsulates sending a move base goal
void sendMoveBaseGoal(geometry_msgs::Pose pose,MoveBaseClient* move_base_client) {
	if (!simulation_mode) {
		ROS_INFO("ROBOT_NAVIGATION Sending move base goal");

		move_base_msgs::MoveBaseGoal move_base_goal;
		move_base_goal.target_pose.header.frame_id = "map";
		move_base_goal.target_pose.header.stamp = ros::Time::now();
		move_base_goal.target_pose.pose=pose;

		move_base_client->sendGoal(move_base_goal);
		ROS_INFO("ROBOT_NAVIGATION goal sent %f %f %f %f %f %f",move_base_goal.target_pose.pose.position.x, 
			move_base_goal.target_pose.pose.position.y, move_base_goal.target_pose.pose.orientation.x ,
			 move_base_goal.target_pose.pose.orientation.y , move_base_goal.target_pose.pose.orientation.z ,
			  move_base_goal.target_pose.pose.orientation.w);	


	}
}

//in the spencer project we use smaller grid maps for navigations associated to couples of symbolic nodes. This procedure
//calls a service to switch grid map on two symbolic nodes.
bool switchMap(string node1, string node2) {
	annotated_mapping::SwitchMap srv;
	srv.request.name_1=node1;
	srv.request.name_2=node2;
	switch_map_client.call(srv);
	
	bool success=srv.response.success;

	return success;
}

 
//moves to: can have a symbolic destination (will plan to reach it), a given symbolic path or a list of coordinates
void moveTo(const supervision_msgs::MoveToGoalConstPtr &goal,MoveToServer* move_to_action_server,
	MoveBaseClient* move_base_client, ros::ServiceClient* calculate_path_client) {
	//supervision will publish on a status topic as well as giving feedback for the actionn	

	supervision_msgs::MoveToFeedback feedback;
	supervision_msgs::MoveToResult result;
	supervision_msgs::SupervisionStatus status_msg; 

	bool symbolic_navigation=false; //this variable is true if we're not just going through a list of coordinates

	if (goal->destination!="") {
		ROS_INFO("ROBOT_NAVIGATION Received destination");
		ROS_INFO("ROBOT_NAVIGATION Received request to move to %s",goal->destination.c_str());
		symbolic_navigation=true;
	}
	else if (goal->path.size()!=0) {
		ROS_INFO("ROBOT_NAVIGATION Received path");
		ROS_INFO("ROBOT_NAVIGATION Received request to move to %s",goal->path[goal->path.size()-1].c_str());
		symbolic_navigation=true;
	}
	else if (goal->coordinates.size()>0) {
		ROS_INFO("ROBOT_NAVIGATION Received coordinates");
		geometry_msgs::Pose last_pose=goal->coordinates[goal->coordinates.size()-1];
		ROS_INFO("ROBOT_NAVIGATION Received request to move to %f %f",last_pose.position.x,last_pose.position.y);
	}
	else {
		ROS_WARN("No path or destination given");
		result.status="no path or destination given";
		move_to_action_server->setAborted(result);
		return;
	}

	string destination=goal->destination;
	string location_destination;
	//get plan for destiantion
	vector<string> nodes; //list of nodes to traverse
	vector<geometry_msgs::Pose> poses; 
	int n_nodes;

	int current_node=0; 


	if (goal->path.size()>0) {
		nodes=goal->path;
		n_nodes=goal->path.size();
	}
	else if (goal->coordinates.size()>0) {
		poses=goal->coordinates;
		n_nodes=goal->coordinates.size();
		current_node=-1;
	}
	else {
		string destination_type=getEntityType(destination);
		ROS_INFO("ROBOT_NAVIGATION got entity type"); 
		if (destination_type!="location") {
			ROS_INFO("ROBOT_NAVIGATION destination is not location"); 
			location_destination=getEntityLocation(destination);
			ROS_INFO("ROBOT_NAVIGATION location is %s",location_destination.c_str());
		}
		else {
			ROS_INFO("ROBOT_NAVIGATION destination is a location");
			location_destination=destination;
		}
		robot_location=getRobotLocation();
		ROS_INFO("ROBOT_NAVIGATION robot location is %s",robot_location.c_str());
			if (robot_location!=location_destination) {
				ROS_INFO("ROBOT_NAVIGATION location different from destination"); 
				supervision_msgs::CalculatePath path_request;
				path_request.request.source=getRobotLocation();
				path_request.request.dest=location_destination;
				if (calculate_path_client->call(path_request)) {
					nodes=path_request.response.path;
				}
				else {
					ROS_ERROR("Failed to calculate path");
					result.status="FAILED";
					result.details="no path found or error";
					move_to_action_server->setAborted(result);
					return;
			}
		}
	}


	ros::Rate r(3); 

	//control variables
	bool task_completed=false;
	bool got_error=false;
	bool is_moving=false;


	if (symbolic_navigation && nodes.size()>0 || poses.size()>0 && !symbolic_navigation) {

		//check if we already have an error
		got_error=check_status->isBatteryLow()||check_status->isBumperPressed() || !ros::ok();


		//switch map to the first one, if we have symbolic navigation
		if (current_node<n_nodes && symbolic_navigation==true) {
			ROS_INFO("ROBOT_NAVIGATION Switching to map %s %s",nodes[current_node].c_str(),nodes[current_node+1].c_str());
			bool switch_map_error=switchMap(nodes[current_node],nodes[current_node+1]);
			if (!switch_map_error) { 
				ROS_INFO("ROBOT_NAVIGATION Error when switching map");
				got_error=true;
			}
		}

		while (!task_completed && !got_error && !move_to_action_server->isPreemptRequested()) {
			geometry_msgs::Pose goal_pose;

			//send the next goal, from the next node center or from the next coordinate.	
			double goal_x,goal_y;
			if (symbolic_navigation==true) {
				geometry_msgs::Pose node_pose=getPose(nodes[current_node+1]);
				goal_x=node_pose.position.x;
				goal_y=node_pose.position.y;

				mutex_location.lock();
				next_area=nodes[current_node+1];
				reached_next_area_=false;
				mutex_location.unlock();

			}
			else {
				goal_x=poses[current_node+1].position.x;
				goal_y=poses[current_node+1].position.y;
			}
			goal_pose.position.x=goal_x;
			goal_pose.position.y=goal_y;	

			goal_pose.orientation.w=1.0;
			
			sendMoveBaseGoal(goal_pose,move_base_client);

			is_moving=true;
			if (symbolic_navigation) {
				ROS_INFO("ROBOT_NAVIGATION Starting to move to %s",nodes[current_node+1].c_str());
				status_msg.status="Moving to "+nodes[current_node+1];
			}
			else {
				ROS_INFO("ROBOT_NAVIGATION Starting to move to %f %f",poses[current_node+1].position.x,
					poses[current_node+1].position.y);
			    status_msg.status="Moving to "+
			    boost::lexical_cast<string>(poses[current_node+1].position.x)+" "+
			    boost::lexical_cast<string>(poses[current_node+1].position.y);
			}
			status_msg.details="";
			status_pub.publish(status_msg);

			bool move_base_error=false;
			bool move_base_arrived=false;
			

			bool node_has_area=false;
			//if the node has a semantic area associated, we will move until we reach the area and not just it.
			if (symbolic_navigation) {
				node_has_area=hasArea(nodes[current_node+1]);
			}

			bool robot_arrived=false;
			//continue until we arrive or have a n error
			while (!got_error && !move_base_error && !move_base_arrived && !robot_arrived && !move_to_action_server->isPreemptRequested()) {
				got_error=check_status->isBatteryLow()||check_status->isBumperPressed() || !ros::ok();
				move_base_error=hasMoveBaseError(move_base_client);
		
				if (symbolic_navigation) {
					mutex_location.lock();
					robot_arrived=reached_next_area_;
					mutex_location.unlock();
				}			
				if (!simulation_mode) {
					move_base_arrived=move_base_client->getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
				}

				r.sleep();
			}

			//when we arrive to the next node, if we use symbolic navigation we stop the robot and switch map.
			if (robot_arrived || move_base_arrived && (current_node+1!=n_nodes-1)) {
				if (!simulation_mode) {
					move_base_client->cancelGoal();
				}
				is_moving=false;

				current_node++;
				if (symbolic_navigation) {
					ROS_INFO("ROBOT_NAVIGATION Reached node %s",nodes[current_node].c_str());
				
					ROS_INFO("ROBOT_NAVIGATION Switching to new map %s %s",nodes[current_node].c_str(),nodes[current_node+1].c_str());
					if(switchMap(nodes[current_node],nodes[current_node+1])){
						feedback.current_node=nodes[current_node];
						move_to_action_server->publishFeedback(feedback);	
					}
					else {
						ROS_INFO("ROBOT_NAVIGATION Error when switching map");
						got_error=true;
					}
				}
				else {
					ROS_INFO("ROBOT_NAVIGATION Reached pose %f %f",poses[current_node].position.x,poses[current_node].position.y);
					feedback.current_pose=poses[current_node];
					move_to_action_server->publishFeedback(feedback);	
				}
			}
			else if (move_base_arrived && (current_node+1)==n_nodes-1) {
				task_completed=true;
			}
		
		}
	}
	else {
		task_completed=true;
	}
	//if we're reaching an entity in a location
	if (task_completed && destination!=location_destination) {
		task_completed=false;

		geometry_msgs::Pose goal_pose=getPose(destination);

		ROS_INFO("ROBOT_NAVIGATION Starting to move to %s",destination.c_str());

		sendMoveBaseGoal(goal_pose,move_base_client);
		is_moving=true;
		
		bool node_has_area=hasArea(destination);
		ROS_INFO("goal pose is %f %f ", goal_pose.position.x,goal_pose.position.y);
		ROS_INFO("node has areas %d",node_has_area);

		bool move_base_error=false;
		bool got_error=false;
		bool move_base_arrived=false;
		bool robot_arrived=false;

		mutex_location.lock();
		next_area=destination;
		reached_next_area_=false;
		mutex_location.unlock();

		while (!got_error && !move_base_error && !move_base_arrived && !robot_arrived && !move_to_action_server->isPreemptRequested()) {
			got_error=check_status->isBatteryLow()||check_status->isBumperPressed() || !ros::ok();
			move_base_error=hasMoveBaseError(move_base_client);
		
			if (hasArea) {
				mutex_location.lock();
				robot_arrived=reached_next_area_;
				mutex_location.unlock();
			}			
			if (!simulation_mode) {
				move_base_arrived=move_base_client->getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
			}

			r.sleep();
		}
		if (move_base_arrived || robot_arrived) task_completed=true;

	}

	//publish final task information
	if (task_completed) {
		status_msg.status="Task Completed";
		status_msg.details="";

		ROS_INFO("ROBOT_NAVIGATION Task completed");
		result.status="OK";
		status_pub.publish(status_msg);
		move_to_action_server->setSucceeded(result);
	}
	else {
		ROS_INFO("ROBOT_NAVIGATION Navigation Task Failed");

		status_msg.status="Task Failed";

		if (move_to_action_server->isPreemptRequested()) {
			ROS_INFO("ROBOT_NAVIGATION Server was preempted");
					status_msg.details="Preempted";
		}
		else if (hasMoveBaseError(move_base_client)) {
			ROS_INFO("ROBOT_NAVIGATION There was an error with move base");
			status_msg.details="Navigation error";
		}
		else {
			status_msg.details=check_status->getCommonErrorString();
			ROS_INFO("ROBOT_NAVIGATION %s",status_msg.details.c_str());
		}
		status_pub.publish(status_msg);

		result.status="FAILURE";
		result.details=status_msg.details;
		if (is_moving) {
			ROS_INFO("ROBOT_NAVIGATION Stopping move base");
			if (!simulation_mode) {
				move_base_client->cancelGoal();
			}
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

//used to get the location of the robot
void agentFactCallback(const situation_assessment_msgs::FactList::ConstPtr& msg) {
	vector<situation_assessment_msgs::Fact> fact_list=msg->fact_list;
	boost::lock_guard<boost::mutex> lock(mutex_location);

	for (int i=0;i<fact_list.size();i++) {
		if (fact_list[i].subject==robot_name && fact_list[i].predicate[0]=="isInArea") {
			robot_areas=fact_list[i].value;
			for (int i=0; i<robot_areas.size();i++) {
				// ROS_INFO("ROBOT_NAVIGATION robot areas %s",robot_areas[i].c_str());
			}
			// ROS_INFO("Next area is %s",next_area.c_str());
			if (std::find(robot_areas.begin(),robot_areas.end(),next_area)!=robot_areas.end()) {
				// ROS_INFO("ROBOT_NAVIGATION Reached next area");
				reached_next_area_=true;
			}
			break;
		}
	}

}

int main(int argc,char** argv) {
	ros::init(argc,argv,"robot_navigation");
	ros::NodeHandle n;

	ROS_INFO("ROBOT_NAVIGATION Starting robot navigation");


	string doc_name, doc_path;   //spencer maps document path

	//get useful parameters
	n.getParam("/robot/name",robot_name);
	n.getParam("supervision/simulation_mode",simulation_mode);
	n.getParam("supervision/use_map_switching",use_map_switching);

	ROS_INFO("ROBOT_NAVIGATION Parameters are:");
	ROS_INFO("ROBOT_NAVIGATION robot name %s",robot_name.c_str());
	ROS_INFO("ROBOT_NAVIGATION simulation_mode %d",simulation_mode);
	ROS_INFO("ROBOT_NAVIGATION use map switching %d",use_map_switching);



	//connect to the action and servers
	check_status=new CheckStatus(n);

	ROS_INFO("ROBOT_NAVIGATION Connecting for switch map client");
	switch_map_client=n.serviceClient<annotated_mapping::SwitchMap>("switch_map");
	if (use_map_switching) {
		switch_map_client.waitForExistence();
		ROS_INFO("ROBOT_NAVIGATION Connected");
	}


	ROS_INFO("ROBOT_NAVIGATION Connecting to calculate path service");
	ros::ServiceClient calculate_path_client=n.serviceClient<supervision_msgs::CalculatePath>("/supervision/calculate_path",true);
	calculate_path_client.waitForExistence();
	ROS_INFO("ROBOT_NAVIGATION Connected\n");


	ROS_INFO("ROBOT_NAVIGATION Connecting to the simple database service");
	simple_database_client=n.serviceClient<situation_assessment_msgs::QueryDatabase>("situation_assessment/query_database",true);
	simple_database_client.waitForExistence();
	ROS_INFO("ROBOT_NAVIGATION Connected");


	ROS_INFO("ROBOT_NAVIGATION Waiting for move_base");
	MoveBaseClient move_base_client("move_base",true);
	if (!simulation_mode) {
		move_base_client.waitForServer();
	}
	ROS_INFO("ROBOT_NAVIGATION connected to move_base");

	ros::Subscriber agent_sub = n.subscribe("situation_assessment/agent_fact_list", 1000, 
	&agentFactCallback);

	ros::Rate r(3);
	ROS_INFO("ROBOT_NAVIGATION Waiting for agent fact list to be published");
	while (agent_sub.getNumPublishers()==0 && ros::ok()) {
		r.sleep();
	}

	// ROS_INFO("ROBOT_NAVIGATION Connecting to has_published");
	// has_published_client=n.serviceClient<situation_assessment_msgs::EmptyRequest>("situation_assessment/has_published",true);
	// has_published_client.waitForExistence();
	// ROS_INFO("ROBOT_NAVIGATION Connected, now waiting for published");
	// situation_assessment_msgs::EmptyRequest srv;
	// if (has_published_client.call(srv)) {
	// 	ROS_INFO("ROBOT_NAVIGATION Done");
	// }
	// else {
	// 	ROS_ERROR("ROBOT_NAVIGATION fail");
	// }

	status_pub=n.advertise<supervision_msgs::SupervisionStatus>("supervision/status",1000);

	MoveToServer move_to_action_server(n,"supervision/move_to",
		boost::bind(&moveTo,_1,&move_to_action_server,&move_base_client, &calculate_path_client),false);
	move_to_action_server.start();
	ROS_INFO("ROBOT_NAVIGATION Started action server MoveTo");
	
	// ros::Duration(5).sleep();

    ROS_INFO("ROBOT_NAVIGATION Ready");


	ros::spin();
	return 0;

}