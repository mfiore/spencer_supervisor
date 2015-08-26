/**
	control_speed_pomdp.cpp
	author: Michelangelo Fiore
*/

#include <robot_guide/control_speed_pomdp.h>

ControlSpeedPomdp::ControlSpeedPomdp(ros::NodeHandle node_handle):PomdpInterface(node_handle) {
	ROS_INFO("Waiting for Control Speed Pomdp");
	pomdp_client_=node_handle_.serviceClient<appl::GetAction>("appl_request/controlSpeed");
	pomdp_client_.waitForExistence();
	ROS_INFO("Connected to Control Speed Pomdp");

	action_mapping_[0]="decelerate";
	action_mapping_[1]="continue";
	action_mapping_[2]="accelerate";

	started_=false;
}

string ControlSpeedPomdp::update(string highest_density, string in_slow_area) {
	string obs=highest_density+in_slow_area;
	string x_state="";

	string command;
	if (!started_) {
		command="start";
		started_=true;
	}
	else {
		command="update";
	}
	return updatePomdp(command,x_state,obs);
}



