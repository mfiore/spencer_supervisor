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

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 

//services
#include <annotated_mapping/SwitchMap.h>

//self includes
#include <robot_navigation/spencer_map.h>

//libraries
#include <spencer_status/check_status.h>

//some useful typedefs
typedef actionlib::SimpleActionServer<supervision_msgs::MoveToAction> MoveToServer;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//services
ros::ServiceClient switch_map_client;


SpencerMap* spencer_map; //used to get the centroid of the sub maps

//location of the robot
string robot_location;

boost::mutex mutex_location;


//parameters
string robot_name;
bool simulation_mode;

map<string, geometry_msgs::Pose> fake_map; //can be useful when we don't want to read the ma pdocuments

CheckStatus *check_status;

//publishers
ros::Publisher status_pub;

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


bool switchMap(string node1, string node2) {
	annotated_mapping::SwitchMap srv;
	srv.request.name_1=node1;
	srv.request.name_2=node2;
	switch_map_client.call(srv);
	
	bool success=srv.response.success;

	// if (success) {
	// 	static tf::TransformBroadcaster br;
	  
	//   	br.sendTransform(tf::StampedTransform(node_transforms[node1+"_"+node2], ros::Time::now(), "occ_map", "map"));
	// }
	return success;
}



void moveTo(const supervision_msgs::MoveToGoalConstPtr &goal,MoveToServer* move_to_action_server,MoveBaseClient* move_base_client) {
	//supervision will publish on a status topic as well as giving feedback for the actionn	
	supervision_msgs::MoveToFeedback feedback;
	supervision_msgs::MoveToResult result;
	supervision_msgs::SupervisionStatus status_msg; 

	bool symbolic_navigation=false;

	if (goal->destination!="") {
		ROS_INFO("Received request to move to %s",goal->destination.c_str());
		symbolic_navigation=true;
	}
	else if (goal->path.size()!=0) {
		ROS_INFO("Received request to move to %s",goal->path[goal->path.size()-1].c_str());
		symbolic_navigation=true;
	}
	else if (goal->coordinates.size()>0) {
		geometry_msgs::Pose last_pose=goal->coordinates[goal->coordinates.size()-1];
		ROS_INFO("Received request to move to %f %f",last_pose.position.x,last_pose.position.y);
	}
	else {
		ROS_WARN("No path or destination given");
		result.status="no path or destination given";
		move_to_action_server->setAborted(result);
	}

	string destination=goal->destination;
	//get plan for destiantion
	vector<string> nodes; //list of nodes to traverse
	vector<geometry_msgs::Pose> poses; 
	int n_nodes;

	int current_node=0; 

	//get plan for destination (TODO)
	if (goal->path.size()!=0) {
		nodes=goal->path;
		n_nodes=goal->path.size();
	}
	else if (destination!=""){

	}
	else {
		poses=goal->coordinates;
		n_nodes=goal->coordinates.size();
		current_node=-1;
	}


	ros::Rate r(3); 

	bool task_completed=false;
	bool got_error=false;
	bool is_moving=false;

	got_error=check_status->isBatteryLow()||check_status->isBumperPressed() || !ros::ok();

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
	
		double goal_x,goal_y;
		if (symbolic_navigation==true) {
			spencer_map->getMapCenter(nodes[current_node+1],&goal_x,&goal_y);
		}
		else {
			goal_x=poses[current_node].position.x;
			goal_y=poses[current_node].position.y;
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

		while (!got_error && !move_base_error && !move_base_arrived && !robot_arrived && !move_to_action_server->isPreemptRequested()) {
			got_error=check_status->isBatteryLow()||check_status->isBumperPressed() || !ros::ok();
			move_base_error=hasMoveBaseError(move_base_client);
			
			if (!simulation_mode) {
				move_base_arrived=move_base_client->getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
			}
			if (!got_error && symbolic_navigation==true) {
				mutex_location.lock();
				robot_arrived=robot_location==nodes[current_node+1] && (current_node+1)!=n_nodes-1;
				mutex_location.unlock();
			}

			r.sleep();
		}

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

	n.getParam("/robot/name",robot_name);
	n.getParam("supervision/simulation_mode",simulation_mode);
	n.getParam("/supervision/doc_path",doc_path);
	n.getParam("/supervision/doc_name",doc_name);

	ROS_INFO("Parameters are:");
	ROS_INFO("robot name %s",robot_name.c_str());
	ROS_INFO("simulation_mode %d",simulation_mode);
	ROS_INFO("doc path %s",doc_path.c_str());
	ROS_INFO("doc name %s",doc_name.c_str());


	//fake map when we don't want to use spencer mapping

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


	spencer_map=new SpencerMap(n,doc_path,doc_name);
	if (!spencer_map->calculateMapInfos()) return 0;


	//connect to the action and servers
	
	check_status=new CheckStatus(n);

	ROS_INFO("Connecting for switch map client");
	switch_map_client=n.serviceClient<annotated_mapping::SwitchMap>("switch_map");
	switch_map_client.waitForExistence();
	ROS_INFO("Connected");

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
		boost::bind(&moveTo,_1,&move_to_action_server,&move_base_client),false);
	move_to_action_server.start();
	ROS_INFO("Started action server MoveTo");
	

	ROS_INFO("Ready");

	ros::spin();
	return 0;

}