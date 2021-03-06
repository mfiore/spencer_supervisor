#include <robot_guide/guide_pomdp.h>

GuidePomdp::GuidePomdp(ros::NodeHandle node_handle):PomdpInterface(node_handle) {
	ROS_INFO("ROBOT_GUIDE Waiting for Guide Pomdp");
	pomdp_client_=node_handle_.serviceClient<appl::GetAction>("appl_request/guide_pomdp");
	pomdp_client_.waitForExistence();
	ROS_INFO("ROBOT_GUIDE Connected to Guide Pomdp");

	action_mapping_[0]="continue";
	action_mapping_[1]="wait";
	action_mapping_[2]="abandon";

	started_=false;
}

string GuidePomdp::update(string timer, string delta_distance, string distance, string orientation, 
	string group_is_moving) {
	string x_state=timer;
	string obs=delta_distance+distance+orientation+group_is_moving;

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



