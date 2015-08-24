/**
	control_speed_pomdp.h
	author: Michelangelo Fiore

	This class provides an interface with the control speed POMDP for group guiding.
	This POMDP takes two observations:
	highest_density: the robot area where there is a higher density of followers (behind\sides)
	in_slow_area: true/false if there is/isn't a follower in the "slow down" robot area

	The class overrides the PomdpInterface class.
*/

#ifndef CONTROL_SPEED_POMDP_H
#define CONTROL_SPEED_POMDP_H

#include <robot_guide/pomdp_interface.h>

#include <ros/ros.h>
#include <string>
#include <map>

using namespace std;
class ControlSpeedPomdp:public PomdpInterface {
	public:
		ControlSpeedPomdp(ros::NodeHandle node_handle);
		string update(string highest_density, string in_slow_area);
		bool started_; //the first time the pomdp is called there is a slightly different syntax.
};

#endif