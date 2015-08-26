/**
	pomdp_interface.cpp

	author Michelangelo Fiore

*/

#include <robot_guide/pomdp_interface.h>

PomdpInterface::PomdpInterface(ros::NodeHandle node_handle):node_handle_(node_handle) {

}


string PomdpInterface::updatePomdp(string command, string x_state, string obs) {
	appl::GetAction srv;
	if (command=="start") {
		srv.request.cmd=1;
	}
	else {
		srv.request.cmd=2;
	}
	srv.request.obs=obs;
	srv.request.xstate=x_state;

	pomdp_client_.call(srv);
	int action=srv.response.action;
	return action_mapping_[action];
}
