/**
	guide_pomdp.h
	author: Michelangelo Fiore

	This class provides an interface with the guide POMDP for group guiding.
	This POMDP takes 5 observations:
	timer: ok/expired used in timed robot events (ex: robot is waiting for people)
	delta_distance:  increasing/decreasing/stable, represents the variation of the group's distance from the robot
	distance: close/far/outOfRange, distance from the group to the robot
	orientation: towardRobot/other orientation of the group to the robot
	group_is_moving: true/false 

	The class overrides the PomdpInterface class.
*/

#ifndef GUIDE_POMDP_H
#define GUIDE_POMDP_H

#include <robot_guide/pomdp_interface.h>

#include <ros/ros.h>
#include <string>
#include <map>

using namespace std;
class GuidePomdp:public PomdpInterface {
	public:
		GuidePomdp(ros::NodeHandle node_handle);
		//updates the pomdp, returning the next action
		string update(string timer, string delta_distance, string distance, string orientation, 
	string group_is_moving);
		bool started_;
};
#endif