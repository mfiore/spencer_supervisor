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
#include <situation_assessment_msgs/QueryDatabase.h>

#include <supervision_msgs/CalculatePath.h>
#include <supervision_msgs/GetConnectedNodes.h>

#include <spencer_control_msgs/SetMaxVelocity.h>


#include <nav_msgs/Path.h>



#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 
#include <boost/lexical_cast.hpp>

//services
#include <annotated_mapping/SwitchMap3.h>
#include <situation_assessment_msgs/QueryDatabase.h>
#include <situation_assessment_msgs/EmptyRequest.h>


#include <robot_navigation/path_length.h>
#include <robot_navigation/path_publisher.h>

//libraries
#include <spencer_status/check_status.h>

//some useful typedefs
typedef actionlib::SimpleActionServer<supervision_msgs::MoveToAction> MoveToServer;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//services
ros::ServiceClient switch_map_client_;
ros::ServiceClient simple_database_client_;
ros::ServiceClient get_connected_nodes_client_;
ros::ServiceClient control_speed_client_;



//location of the robot
vector<string> robot_areas_;
geometry_msgs::Pose robot_pose_;
boost::mutex mutex_location_;
boost::mutex mutex_robot_pose_;



//parameters
string robot_name_;
bool simulation_mode_;
bool use_map_switching_;
double switch_map_sleep_;
bool use_control_speed_;
double starting_speed_;
double angular_velocity_;
bool use_rolling_window_;
double rolling_threshold_;


CheckStatus *check_status_; //contains the symbolic status of the robot's system (batteries, bumper, emergency switch..)

//publishers
ros::Publisher status_pub_;
ros::Publisher path_pub_;

int max_move_base_errors_;

double previous_length_;
vector<string> old_path_;



geometry_msgs::Pose getRobotPose() {
	boost::lock_guard<boost::mutex> lock(mutex_robot_pose_);
	return robot_pose_;
}

void setRobotPose(geometry_msgs::Pose pose) {
	boost::lock_guard<boost::mutex> lock(mutex_robot_pose_);
	robot_pose_=pose;
}

void setRobotAreas(vector<string> location) {
	boost::lock_guard<boost::mutex> lock(mutex_location_);
	robot_areas_=location;

}

vector<string> getRobotAreas()  {
	boost::lock_guard<boost::mutex> lock(mutex_location_);
	return robot_areas_;
}



//returns true if there is a move base error. 
bool hasMoveBaseError(MoveBaseClient* move_base_client) {
	if (!simulation_mode_) {
				return move_base_client->getState()==actionlib::SimpleClientGoalState::ABORTED || 
				move_base_client->getState()==actionlib::SimpleClientGoalState::LOST ||
				move_base_client->getState()==actionlib::SimpleClientGoalState::PREEMPTED;
	}
		else return false;
}

bool hasSystemError(CheckStatus* check_status_) {
  if (check_status_->isBatteryLow() || check_status_->isStopped()) {
     ROS_INFO("ROBOT_NAVIGATION battery low %d",check_status_->isBatteryLow());
      ROS_INFO("ROBOT_NAVIGATION is stopped %d",check_status_->isStopped());
  }
  
  return  check_status_->isBatteryLow() || check_status_->isStopped() || !ros::ok();
}

bool isPaused(CheckStatus* check_status_) {
	return check_status_->isPaused() || check_status_->isBumperPressed();
}


//incapsulates sending a move base goal
void sendMoveBaseGoal(geometry_msgs::Pose pose,MoveBaseClient* move_base_client) {
	if (!simulation_mode_) {
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
bool switchMap(string node1, string node2, string node3) {
	ROS_INFO("Switching map to %s %s %s",node1.c_str(),node2.c_str(),node3.c_str());
	if (use_map_switching_) {
		annotated_mapping::SwitchMap3 srv;
		srv.request.name_1=node1;
		srv.request.name_2=node2;
		srv.request.name_3=node3;
		if (switch_map_client_.call(srv)) {
			bool success=srv.response.success;
			if (!success) {
				ROS_ERROR("ROBOT_NAVIGATION failed to switch map");
			} 
			ros::Duration(switch_map_sleep_).sleep();
			return success;
		}
		else {
			ROS_ERROR("ROBOT_NAVIGATION failed to switch map");
		}
	}
	else return true;
}

string getOtherLinkedNode(string node, string different_node ) {
	supervision_msgs::GetConnectedNodes srv;
	srv.request.node=node;
	if (get_connected_nodes_client_.call(srv)) {
		vector<string> connected_nodes=srv.response.connected_nodes;
		for (int i=0; i<connected_nodes.size();i++) {
			if (connected_nodes[i]!=different_node) return connected_nodes[i];
		}
	}
	else {
		ROS_ERROR("ROBOT_NAVIGATION can't contact get connected nodes");
		return "";
	}
	return "";
}

bool switchMapHelper(vector<string> path, int current_node) {
	//try to get previous node
	string previous_node;
	string node1,node2,node3;
	//if it's the first node of the path, take the previous, if it exists, and the following.
	//If the previous doesn't exist 
	if (path.size()>=3) { 	//there are enough nodes in the path for a map switch
		if (current_node!=0) {   //we simply take the three nodes in the path
			node1=path[current_node-1];
			node2=path[current_node];
			node3=path[current_node+1];
		}
		else { //we look if there is a previous node than current. If there isn't we take a big map starting from the first node to the 
			//others
			string previous_node=getOtherLinkedNode(path[current_node],path[current_node+1]);
			if (previous_node!="") { 
				node1=previous_node;
				node2=path[current_node];
				node3=path[current_node+1];
			}
			else {
				node1=path[current_node];
				node2=path[current_node+1];
				node3=path[current_node+2];
			}
		}
	}
	else if (path.size()==2){ //2 nodes path
		string previous_node=getOtherLinkedNode(path[current_node],path[current_node+1]);
		if (previous_node!="") {
			node1=previous_node;
			node2=path[current_node];
			node3=path[current_node+1];
		}
		else {
			string next_node=getOtherLinkedNode(path[current_node+1],path[current_node]);
			if (next_node!="") {
				node1=path[current_node];
				node2=path[current_node+1];
				node3=next_node;
			}
			else return false;
		}
	}
	else {
		node1=getOtherLinkedNode(path[current_node],"");
		node3=getOtherLinkedNode(path[current_node],previous_node);
		node2=path[current_node];
	}
	return switchMap(node1,node2,node3);

}


double dist2d(geometry_msgs::Pose p1,geometry_msgs::Pose p2) {
	return sqrt(pow(p2.position.y-p1.position.y,2)+pow(p2.position.x-p1.position.x,2));
}

bool moveToNext(geometry_msgs::Pose goal_pose, MoveBaseClient *move_base_client, MoveToServer* move_to_action_server,
	bool should_check_area,	string destination,
	bool should_check_rolling_window_node, geometry_msgs::Pose rolling_window_node,
	 bool symbolic_navigation, DatabaseQueries *database_queries) {
	supervision_msgs::SupervisionStatus status_msg; 
	ros::Rate r(3); 

	sendMoveBaseGoal(goal_pose,move_base_client);

	ROS_INFO("ROBOT_NAVIGATION should check area is %d",should_check_area);
	ROS_INFO("ROBOT_NAVIGATION should check rolling window node is %d",should_check_rolling_window_node);
	bool is_moving=true;
	if (symbolic_navigation) {
		ROS_INFO("ROBOT_NAVIGATION Starting to move to %s",destination.c_str());
		ROS_INFO("ROBOT_NAVIGATION Goal pose is %f %f",goal_pose.position.x,goal_pose.position.y);
		status_msg.details="Moving to "+destination;
	}
	else {
		ROS_INFO("ROBOT_NAVIGATION Starting to move to %f %f",goal_pose.position.x,
		goal_pose.position.y);
	    status_msg.details="Moving to "+
	    boost::lexical_cast<string>(goal_pose.position.x)+" "+
	    boost::lexical_cast<string>(goal_pose.position.y);
	}
	status_msg.status="RUNNING";
	status_pub_.publish(status_msg);

	bool move_base_error=false;
	bool move_base_arrived=false;
	//if the node has a semantic area associated, we will move until we reach the area and not just it.
	//and if the should check this area (meaning no rolling window or last location and rolling window)
	if (symbolic_navigation) {
		bool node_has_area=database_queries->hasArea(destination);
		if (!node_has_area) {
			ROS_INFO("ROBOT_NAVIGATION node doesn't have an area so we will drive to the position");
			should_check_area=false;
		}
	}

	bool robot_arrived=false;
	//continue until we arrive or have an error
	while (!hasSystemError(check_status_) && 
		!hasMoveBaseError(move_base_client) && !move_base_arrived && 
		!robot_arrived && !move_to_action_server->isPreemptRequested())
		 {
	
		if (!simulation_mode_) {
			move_base_arrived=move_base_client->getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
		}

		if (should_check_rolling_window_node) {
			geometry_msgs::Pose robot_pose=getRobotPose();
			if (dist2d(robot_pose,rolling_window_node)<rolling_threshold_) {
				robot_arrived=true;
			} 
		}
		if (should_check_area) {
			vector<string> robot_areas=getRobotAreas();
			if (std::find(robot_areas_.begin(),robot_areas_.end(),destination)!=robot_areas_.end()) {
				robot_arrived=true;
			}
		}

		if (check_status_->isPaused()) {
			ROS_INFO("ROBOT_NAVIGATION Pausing");
			if (is_moving && !simulation_mode_) {
				move_base_client->cancelGoal();
				is_moving=false;
			}
			while (check_status_->isPaused() && !hasSystemError(check_status_) && !move_to_action_server->isPreemptRequested()) {
				status_msg.status="supervision is paused";
				status_pub_.publish(status_msg);
				r.sleep();
			}
			if (!hasSystemError(check_status_) && !move_to_action_server->isPreemptRequested()) {
			   ROS_INFO("ROBOT_NAVIGATION Resuming");
			   sendMoveBaseGoal(goal_pose,move_base_client);
			}
			status_msg.status="resuming movement";
			status_pub_.publish(status_msg);
		}
		else if (check_status_->isPlannerBlocked()) {
			ROS_INFO("Planner blocked");
			// move_base_client->cancelGoal();
			// is_moving=false;

			while (check_status_->isPlannerBlocked() && !hasSystemError(check_status_) 
				 && !hasMoveBaseError(move_base_client)
				 && !move_to_action_server->isPreemptRequested()) {
				// sc.say("Excuse me, could you let me pass?");
				r.sleep();
			}
			ROS_INFO("Not blocked anymore");
		}
		r.sleep();
	}
	ROS_INFO("has system error is %d",hasSystemError(check_status_));
	ROS_INFO("has move base error is %d",hasMoveBaseError(move_base_client));
	ROS_INFO("move base arrived is %d",move_base_arrived);
	ROS_INFO("robot arrived is %d",robot_arrived);
	ROS_INFO("Preempt request is %d",move_to_action_server->isPreemptRequested());


	return robot_arrived || move_base_arrived;
		
}
 

bool isSamePath(vector<string> path) {
	int j=old_path_.size()-1;
	int i=path.size()-1;

	if (old_path_.size()==0) return false;

	while (i>=0 && j>=0) {

		if (path[i]!=old_path_[j]) return false;
		i--;
		j--;
	}
	return true;
}

void slowAndStop(MoveBaseClient* move_base_client) {
	ROS_INFO("ROBOT_NAVIGATION slowing down and stopping");
	double time=0.5;
	if (!simulation_mode_) {
		if (use_control_speed_) {
			spencer_control_msgs::SetMaxVelocity srv;
			srv.request.max_linear_velocity=0.3;
			srv.request.max_angular_velocity=0.4;

			control_speed_client_.call(srv);
			ros::Duration(time).sleep();
			srv.request.max_linear_velocity=0.1;

			control_speed_client_.call(srv);
			ros::Duration(time).sleep();
		}
		move_base_client->cancelGoal();
		if (use_control_speed_) {
			spencer_control_msgs::SetMaxVelocity srv;
			srv.request.max_linear_velocity=starting_speed_;
			srv.request.max_angular_velocity=angular_velocity_;

			control_speed_client_.call(srv);
		}

	}
}



int getFurthestNode(vector<geometry_msgs::Pose> node_poses) {
	int i=0;
	geometry_msgs::Pose robot_pose=getRobotPose();

	ROS_INFO("Node poses size %ld",node_poses.size());
	while (i<node_poses.size()) {
		double dist=dist2d(robot_pose,node_poses[i]);
		ROS_INFO("ROBOT_NAVIGATION dist is %f",dist);
		if (dist<rolling_threshold_) {
			i++;
		}
		else {
			break;
		}
	}
	return i-1;
} 


//moves to: can have a symbolic destination (will plan to reach it), a given symbolic path or a list of coordinates
void moveTo(const supervision_msgs::MoveToGoalConstPtr &goal,MoveToServer* move_to_action_server,
	MoveBaseClient* move_base_client, ros::ServiceClient* calculate_path_client, DatabaseQueries* database_queries,
	PathLength* path_length, PathPublisher* path_publisher) {
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
		ROS_WARN("ROBOT_NAVIGATION No path or destination given");
		result.status="FAILED";
		result.details="no path or destination given";
		move_to_action_server->setAborted(result);
		return;
	}

	string destination=goal->destination;
	string location_destination;
	string robot_location;
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
		string destination_type=database_queries->getEntityType(destination);
		ROS_INFO("ROBOT_NAVIGATION got entity type"); 
		if (destination_type!="location") {
			ROS_INFO("ROBOT_NAVIGATION destination is not location"); 
			location_destination=database_queries->getEntityLocation(destination);
			ROS_INFO("ROBOT_NAVIGATION location is %s",location_destination.c_str());
		}
		else {
			ROS_INFO("ROBOT_NAVIGATION destination is a location");
			location_destination=destination;
		}
		robot_location=database_queries->getRobotLocation(robot_areas_);
		ROS_INFO("ROBOT_NAVIGATION robot location is %s",robot_location.c_str());
			if (robot_location!=location_destination) {
				ROS_INFO("ROBOT_NAVIGATION location different from destination"); 
				supervision_msgs::CalculatePath path_request;
				path_request.request.source=database_queries->getRobotLocation(robot_areas_);
				path_request.request.dest=location_destination;
				if (calculate_path_client->call(path_request)) {
					ROS_INFO("ROBOT_NAVIGATION calculated path to destination");
					nodes=path_request.response.path;
					n_nodes=nodes.size();
				}
				else {
					ROS_ERROR("ROBOT_NAVIGATION Failed to calculate path");
					result.status="FAILED";
					result.details="no path found or error";
					move_to_action_server->setAborted(result);
					return;
			}
		}
	}
	vector<geometry_msgs::Pose> node_poses;


	ROS_INFO("Before calculate positions");
	//calculate position of nodes
	for (int i=0; i<nodes.size();i++) {
		geometry_msgs::Pose pose=database_queries->getPose(nodes[i]);
		node_poses.push_back(pose);
	}

	bool no_locations=false;
	ROS_INFO("Before check destination");
	if (destination!=location_destination) {
		geometry_msgs::Pose pose=database_queries->getPose(destination);
		if (nodes.size()==0) {
		  no_locations=true;
		  geometry_msgs::Pose starting_pose=getRobotPose(); //fake pose for starting condition (since we move to the next node usually)
		  node_poses.push_back(starting_pose);
		  nodes.push_back(robot_location);
		}
		node_poses.push_back(pose);
		nodes.push_back(destination);
	}

	//publish path

	boost::thread t_path(boost::bind(&PathPublisher::publishPath,path_publisher,node_poses,no_locations));

	ROS_INFO("Before same path");
	if (!isSamePath(nodes)) {
		previous_length_=-1;
	}

	ROS_INFO("Before thread");
	boost::thread t_length(boost::bind(&PathLength::startPublishingPath,path_length,node_poses,previous_length_));

	//control variables
	bool task_completed=false;
	bool got_error=false;
	bool is_moving=false;

	got_error=hasSystemError(check_status_);
	n_nodes=node_poses.size();

	ROS_INFO("ROBOT_NAVIGATION Got error is %d",got_error);

	if (node_poses.size()>0 && use_rolling_window_) {
		current_node=getFurthestNode(node_poses)-1;
		ROS_INFO("ROBOT_NAVIGATION furthest node is %d",current_node); 
	} 
	if (symbolic_navigation && nodes.size()>0 || poses.size()>0 && !symbolic_navigation) {
		//switch map to the first one, if we have symbolic navigation
		int n_move_base_errors=0;
		while (!task_completed && !got_error && !move_to_action_server->isPreemptRequested()) {
			if (
				(current_node<n_nodes-1 && destination==location_destination ||  //account for possible last node, which 
				current_node<n_nodes-2 && destination!=location_destination)    // doesn't require map switch
				&& symbolic_navigation==true && !use_rolling_window_) {
				

				bool switch_map_error=switchMapHelper(nodes,current_node);
				if (!switch_map_error) { 
					ROS_INFO("ROBOT_NAVIGATION Error when switching map");
					got_error=true;
					break;
				}
			}
			ROS_INFO("ROBOT_NAVIGATION going for next position");
			geometry_msgs::Pose goal_pose;

			//send the next goal, from the next node center or from the next coordinate.	
			double goal_x,goal_y;
			geometry_msgs::Pose rolling_window_node;
			bool should_check_rolling_window_node=false;
			bool should_check_area=false;

   			string next_area=nodes[current_node+1];
			//assign rolling window and check area information
			if (use_rolling_window_ && symbolic_navigation==true) {
				ROS_INFO("ROBOT_NAVIGATION current node is %d and n_nodes is %d",current_node,n_nodes);
				if (current_node+1<n_nodes-1) {
					rolling_window_node=node_poses[current_node+1];
					should_check_rolling_window_node=true;
				}
				else  {
					ROS_INFO("ROBOT_NAVIGATION we're at the last node, and so will move until the area");
					should_check_area=true;
				}
			}
			else if (symbolic_navigation==true){
				should_check_area=true;
			}

			//get next pose
			if (symbolic_navigation==true) {
				geometry_msgs::Pose node_pose=node_poses[current_node+1];
				goal_x=node_pose.position.x;
				goal_y=node_pose.position.y;
			}
			else {
				goal_x=poses[current_node+1].position.x;
				goal_y=poses[current_node+1].position.y;
			}
			goal_pose.position.x=goal_x;
			goal_pose.position.y=goal_y;	

			goal_pose.orientation.w=1.0;



			bool move_status=moveToNext(goal_pose,move_base_client,move_to_action_server,
				should_check_area,next_area,
				should_check_rolling_window_node,rolling_window_node,
				symbolic_navigation,database_queries
				);
			got_error=!move_status;
			//when we arrive to the next node, if we use symbolic navigation we stop the robot and switch map.
			if (move_status) {
				if (!simulation_mode_ ) {
					if (current_node==n_nodes-2) {
						slowAndStop(move_base_client);
					}
					else {
						move_base_client->cancelGoal();
					}
				}
				is_moving=false;
				current_node++;
				path_length->updateCurrentNode(current_node);
				if (symbolic_navigation) {
					ROS_INFO("ROBOT_NAVIGATION Reached node %s",nodes[current_node].c_str());
				}
				else {
					ROS_INFO("ROBOT_NAVIGATION Reached pose %f %f",poses[current_node].position.x,poses[current_node].position.y);
					feedback.current_pose=poses[current_node];
					move_to_action_server->publishFeedback(feedback);	
				}
				if (current_node==n_nodes-1) {
					ROS_INFO("ROBOT_NAVIGATION Reached last node");
					task_completed=true;
				}
			}
			else {
				if (hasMoveBaseError(move_base_client)) {
					if (n_move_base_errors<max_move_base_errors_) {
						got_error=false;
					}
					ROS_INFO("ROBOT_NAVIGATION received move base error: %d of %d maximum errors",n_move_base_errors,max_move_base_errors_);
				}
				if (!simulation_mode_) {
			   		 move_base_client->cancelGoal();
			  	}
			}
		} 		
	}
	else {
	  ROS_INFO("ROBOT_NAVIGATION task completed because robot is already there");
		task_completed=true; //if there are no nodes in the path we're already arrived
	}
	
	slowAndStop(move_base_client);
    //if (!simulation_mode_) {
//		move_base_client->cancelGoal();
//	}

	previous_length_=path_length->getTotalLength();
	old_path_=nodes;

	//publish final task information
	path_length->stopPublishingPath();
	path_publisher->setShouldPublish(false);
	t_length.join();
	t_path.join();

	if (task_completed) {
		status_msg.status="COMPLETED";
		status_msg.details="";

		ROS_INFO("ROBOT_NAVIGATION Task completed");
		result.status="COMPLETED";
		status_pub_.publish(status_msg);
		move_to_action_server->setSucceeded(result);
		status_pub_.publish(status_msg);
	}
	else {
		ROS_INFO("ROBOT_NAVIGATION Navigation Task Failed");

		status_msg.status="FAILED";

		if (move_to_action_server->isPreemptRequested()) {
			ROS_INFO("ROBOT_NAVIGATION Server was preempted");
					status_msg.details="Preempted";
		}
		else if (hasMoveBaseError(move_base_client)) {
			ROS_INFO("ROBOT_NAVIGATION There was an error with move base");
			status_msg.details="Navigation error";
		}
		else {
			status_msg.details=check_status_->getCommonErrorString();
			ROS_INFO("ROBOT_NAVIGATION %s",status_msg.details.c_str());
		}
		status_pub_.publish(status_msg);

		result.status="FAILED";
		result.details=status_msg.details;
		if (is_moving) {
			ROS_INFO("ROBOT_NAVIGATION Stopping move base");
			if (!simulation_mode_) {
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

	for (int i=0;i<fact_list.size();i++) {
		if (fact_list[i].predicate.size()>0) {
			if (fact_list[i].subject==robot_name_ && fact_list[i].predicate[0]=="isInArea") {
				setRobotAreas(fact_list[i].value);
			}
			else if (fact_list[i].subject==robot_name_ && fact_list[i].predicate[0]=="pose") {
				if (fact_list[i].value.size()>2) {
					double robot_x=boost::lexical_cast<double>(fact_list[i].value[0]);
					double robot_y=boost::lexical_cast<double>(fact_list[i].value[1]);
					geometry_msgs::Pose pose;
					pose.position.x=robot_x;
					pose.position.y=robot_y;
					setRobotPose(pose);
					}
				else {
					ROS_WARN("ROBOT_NAVIGATION not enough values for robot pose");
				}
			}
		}
		else {
			ROS_ERROR("ROBOT_NAVIGATION received fact with predicate size 0");
			ROS_ERROR("ROBOT_NAVIGATION Model %s and subject %s",fact_list[i].model.c_str(), fact_list[i].subject.c_str());
		}
	}
}


int main(int argc,char** argv) {
	ros::init(argc,argv,"robot_navigation");
	ros::NodeHandle n;

	ROS_INFO("ROBOT_NAVIGATION Starting robot navigation");


	string doc_name, doc_path;   //spencer maps document path

	//get useful parameters
	n.getParam("/robot/name",robot_name_);
	n.getParam("supervision/simulation_mode",simulation_mode_);
	n.getParam("supervision/use_map_switching",use_map_switching_);
	n.getParam("supervision/max_move_base_errors",max_move_base_errors_);
	n.getParam("supervision/switch_map_sleep",switch_map_sleep_);
	n.getParam("supervision/use_control_speed",use_control_speed_);
	n.getParam("/supervision/starting_speed",starting_speed_);
	n.getParam("/supervision/angular_velocity",angular_velocity_);
	n.getParam("/supervision/use_rolling_window",use_rolling_window_);
	n.getParam("/supervision/rolling_threshold",rolling_threshold_);


	ROS_INFO("ROBOT_NAVIGATION Parameters are:");
	ROS_INFO("ROBOT_NAVIGATION robot name %s",robot_name_.c_str());
	ROS_INFO("ROBOT_NAVIGATION simulation_mode %d",simulation_mode_);
	ROS_INFO("ROBOT_NAVIGATION use map switching %d",use_map_switching_);
	ROS_INFO("ROBOT_NAVIGATION max move base errors %d",max_move_base_errors_);
	ROS_INFO("ROBOT_NAVIGATION use control speed is %d",use_control_speed_);
	ROS_INFO("ROBOT_NAVIGATION switch map sleep is %f", switch_map_sleep_);
	ROS_INFO("ROBOT_NAVIGATION starting speed is %f",starting_speed_);
	ROS_INFO("ROBOT_NAVIGATION angular velocity is %f",angular_velocity_);
	ROS_INFO("ROBOT_NAVIGATION use rolling window is %d",use_rolling_window_);
	ROS_INFO("ROBOT_NAVIGATION rolling threshold %f",rolling_threshold_);



	//connect to the action and servers
	check_status_=new CheckStatus(n);

	ROS_INFO("ROBOT_NAVIGATION Connecting for switch map client");
	switch_map_client_=n.serviceClient<annotated_mapping::SwitchMap3>("switch_map");
	if (use_map_switching_) {
		switch_map_client_.waitForExistence();
		ROS_INFO("ROBOT_NAVIGATION Connected");
	}

	ROS_INFO("ROBOT_NAVIGATION Connecting to control speed");
	control_speed_client_=n.serviceClient<spencer_control_msgs::SetMaxVelocity>("/spencer/control/set_max_velocity",true);
	if (use_control_speed_ && !simulation_mode_) {
		control_speed_client_.waitForExistence();
	}




	ROS_INFO("ROBOT_NAVIGATION Connecting to calculate path service");
	ros::ServiceClient calculate_path_client=n.serviceClient<supervision_msgs::CalculatePath>("/supervision/calculate_path",true);
	calculate_path_client.waitForExistence();
	ROS_INFO("ROBOT_NAVIGATION Connected\n");

	DatabaseQueries database_queries(n,robot_name_);


	ROS_INFO("ROBOT_NAVIGATION Waiting for move_base");
	MoveBaseClient move_base_client("move_base",true);
	if (!simulation_mode_) {
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


	ROS_INFO("ROBOT_NAVIGATION waiting for get connected nodes");
	get_connected_nodes_client_=n.serviceClient<supervision_msgs::GetConnectedNodes>("supervision/get_connected_nodes",true);
	get_connected_nodes_client_.waitForExistence();
	ROS_INFO("ROBOT_NAVIGATION connected to get connected nodes");
	PathLength path_length(n,&database_queries);
	PathPublisher path_publisher(n);

	status_pub_=n.advertise<supervision_msgs::SupervisionStatus>("supervision/navigation/status",1000);



	MoveToServer move_to_action_server(n,"supervision/move_to",
		boost::bind(&moveTo,_1,&move_to_action_server,&move_base_client, &calculate_path_client,&database_queries,
			&path_length,&path_publisher),false);
	move_to_action_server.start();
	ROS_INFO("ROBOT_NAVIGATION Started action server MoveTo");
	
	supervision_msgs::SupervisionStatus status_msg; 

	status_msg.status="IDLE";
	// while (ros::ok()) {
		status_pub_.publish(status_msg);
	// 	r.sleep();
	// }
    ROS_INFO("ROBOT_NAVIGATION Ready");


	ros::spin();
	return 0;

}
