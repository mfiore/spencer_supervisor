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
#include <situation_assessment_msgs/GetMap.h>
#include <situation_assessment_msgs/Graph.h>
#include <situation_assessment_msgs/Node.h>
#include <supervision_msgs/CalculatePath.h>


#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 

//services
#include <annotated_mapping/SwitchMap.h>


//libraries
#include <spencer_status/check_status.h>

//some useful typedefs
typedef actionlib::SimpleActionServer<supervision_msgs::MoveToAction> MoveToServer;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//services
ros::ServiceClient switch_map_client;


//location of the robot
string robot_location;
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


//gets the symbolic map, in order to take the node centers
bool getMap(ros::ServiceClient* get_symbolic_map_client) {
	situation_assessment_msgs::GetMap get_map_request;
	if (!get_symbolic_map_client->call(get_map_request)) {
		return false;
	}
	else {
		situation_assessment_msgs::Graph g=get_map_request.response.graph;
		vector<situation_assessment_msgs::Node> node_list=g.nodes;
		for (int i=0; i<node_list.size();i++) {
			situation_assessment_msgs::Node n=node_list[i];
			node_centers_[n.label]=n.center;

		}
	}
	return true;
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
		ROS_INFO("Received destination");
		ROS_INFO("Received request to move to %s",goal->destination.c_str());
		symbolic_navigation=true;
	}
	else if (goal->path.size()!=0) {
		ROS_INFO("Received path");
		ROS_INFO("Received request to move to %s",goal->path[goal->path.size()-1].c_str());
		symbolic_navigation=true;
	}
	else if (goal->coordinates.size()>0) {
		ROS_INFO("Received coordinates");
		geometry_msgs::Pose last_pose=goal->coordinates[goal->coordinates.size()-1];
		ROS_INFO("Received request to move to %f %f",last_pose.position.x,last_pose.position.y);
	}
	else {
		ROS_WARN("No path or destination given");
		result.status="no path or destination given";
		move_to_action_server->setAborted(result);
		return;
	}

	string destination=goal->destination;
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
		supervision_msgs::CalculatePath path_request;
		path_request.request.source=robot_location;
		path_request.request.dest=destination;
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


	ros::Rate r(3); 

	//control variables
	bool task_completed=false;
	bool got_error=false;
	bool is_moving=false;

	//check if we already have an error
	got_error=check_status->isBatteryLow()||check_status->isBumperPressed() || !ros::ok();

	//switch map to the first one, if we have symbolic navigation
	if (current_node<n_nodes && symbolic_navigation==true) {
		ROS_INFO("Switching to map %s %s",nodes[current_node].c_str(),nodes[current_node+1].c_str());
		bool switch_map_error=switchMap(nodes[current_node],nodes[current_node+1]);
		if (!switch_map_error) { 
			ROS_INFO("Error when switching map");
			got_error=true;
		}
	}

	while (!task_completed && !got_error && !move_to_action_server->isPreemptRequested()) {
		geometry_msgs::Pose goal_pose;

		//send the next goal, from the next node center or from the next coordinate.	
		double goal_x,goal_y;
		if (symbolic_navigation==true) {
			geometry_msgs::Point center=node_centers_[nodes[current_node+1]];
			goal_x=center.x;
			goal_y=center.y;
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
			ROS_INFO("Starting to move to %s",nodes[current_node+1].c_str());
			status_msg.status="Moving to "+nodes[current_node+1];
		}
		else {
			ROS_INFO("Starting to move to %f %f",poses[current_node+1].position.x,
				poses[current_node+1].position.y);
		    status_msg.status="Moving to "+
		    boost::lexical_cast<string>(poses[current_node+1].position.x)+" "+
		    boost::lexical_cast<string>(poses[current_node+1].position.y);
		}
		status_msg.details="";
		status_pub.publish(status_msg);

		bool move_base_error=false;
		bool move_base_arrived=false;
		bool robot_arrived=false;

		//continue until we arrive or have a n error
		while (!got_error && !move_base_error && !move_base_arrived && !robot_arrived && !move_to_action_server->isPreemptRequested()) {
			got_error=check_status->isBatteryLow()||check_status->isBumperPressed() || !ros::ok();
			move_base_error=hasMoveBaseError(move_base_client);
			
			if (!simulation_mode) {
				move_base_arrived=move_base_client->getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
			}

			//in symbolic node we arrive as soon as we reach the next node, not waiting to reach the move base destination
			if (!got_error && symbolic_navigation==true) { 
				mutex_location.lock();
				robot_arrived=robot_location==nodes[current_node+1] && (current_node+1)!=n_nodes-1;
				mutex_location.unlock();
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
				ROS_INFO("Reached node %s",nodes[current_node].c_str());
			
				ROS_INFO("Switching to new map %s %s",nodes[current_node].c_str(),nodes[current_node+1].c_str());
				if(switchMap(nodes[current_node],nodes[current_node+1])){
					feedback.current_node=nodes[current_node];
					move_to_action_server->publishFeedback(feedback);	
				}
				else {
					ROS_INFO("Error when switching map");
					got_error=true;
				}
			}
			else {
				ROS_INFO("Reached pose %f %f",poses[current_node].position.x,poses[current_node].position.y);
				feedback.current_pose=poses[current_node];
				move_to_action_server->publishFeedback(feedback);	
			}
		}
		else if (move_base_arrived && (current_node+1)==n_nodes-1) {
			task_completed=true;
		}
	
	}

	//publish final task information
	if (task_completed) {
		status_msg.status="Task Completed";
		status_msg.details="";

		ROS_INFO("Task completed");
		result.status="OK";
		status_pub.publish(status_msg);
		move_to_action_server->setSucceeded(result);
	}
	else {
		ROS_INFO("Navigation Task Failed");

		status_msg.status="Task Failed";

		if (move_to_action_server->isPreemptRequested()) {
					status_msg.details="Preempted";
		}
		else if (move_base_client) {
			status_msg.details="Navigation error";
		}
		else {
			status_msg.details=check_status->getCommonErrorString();
		}
		status_pub.publish(status_msg);

		result.status="FAILURE";
		result.details=status_msg.details;
		if (is_moving) {
			ROS_INFO("Supervision is stopped");
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
			robot_location=fact_list[i].value;
			break;
		}
	}

}

int main(int argc,char** argv) {
	ros::init(argc,argv,"robot_navigation");
	ros::NodeHandle n;

	ROS_INFO("Starting robot navigation");


	string doc_name, doc_path;   //spencer maps document path

	//get useful parameters
	n.getParam("/robot/name",robot_name);
	n.getParam("supervision/simulation_mode",simulation_mode);
	n.getParam("supervision/use_map_switching",use_map_switching);

	ROS_INFO("Parameters are:");
	ROS_INFO("robot name %s",robot_name.c_str());
	ROS_INFO("simulation_mode %d",simulation_mode);
	ROS_INFO("use map switching %d",use_map_switching);



	//connect to the action and servers
	check_status=new CheckStatus(n);

	ROS_INFO("Connecting for switch map client");
	switch_map_client=n.serviceClient<annotated_mapping::SwitchMap>("switch_map");
	if (use_map_switching) {
		switch_map_client.waitForExistence();
		ROS_INFO("Connected");
	}


	ROS_INFO("Connecting to calculate path service");
	ros::ServiceClient calculate_path_client=n.serviceClient<supervision_msgs::CalculatePath>("/supervision/calculate_path",true);
	calculate_path_client.waitForExistence();
	ROS_INFO("Connected\n");

	ROS_INFO("Connecting to get map service");
		ros::ServiceClient get_symbolic_map_client=n.serviceClient<situation_assessment_msgs::GetMap>("/situation_assessment/get_symbolic_map",true);
		get_symbolic_map_client.waitForExistence();
		ROS_INFO("Connected\n");

	if (!getMap(&get_symbolic_map_client)) {
		ROS_ERROR("Can't get map");
		ros::shutdown();
	}



	ROS_INFO("Waiting for move_base");
	MoveBaseClient move_base_client("move_base",true);
	if (!simulation_mode) {
		move_base_client.waitForServer();
	}
	ROS_INFO("connected to move_base");

	ros::Subscriber agent_sub = n.subscribe("situation_assessment/agent_fact_list", 1000, 
	&agentFactCallback);

	ros::Rate r(3);
	ROS_INFO("Waiting for agent fact list to be published");
	while (agent_sub.getNumPublishers()==0 && ros::ok()) {
		r.sleep();
	}



	status_pub=n.advertise<supervision_msgs::SupervisionStatus>("supervision/status",1000);

	MoveToServer move_to_action_server(n,"supervision/move_to",
		boost::bind(&moveTo,_1,&move_to_action_server,&move_base_client, &calculate_path_client),false);
	move_to_action_server.start();
	ROS_INFO("Started action server MoveTo");
	

	ROS_INFO("Ready");

	ros::spin();
	return 0;

}