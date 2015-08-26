/**
  pomdp_interface.h
  Author: Michelangelo Fiore
 
  super class of the pomdp used in the supervisor.  	
 */

#ifndef PODMP_INTERFACE_H
#define PODMP_INTERFACE_H

#include <ros/ros.h>
#include <string>
#include <map>
#include <appl/GetAction.h>

using namespace std;

class PomdpInterface {
	public:

	protected:
		PomdpInterface(ros::NodeHandle node_handle);
		//pomdps return actions as numbers but the supervision system uses strings instead. This variable will be 
		//filled by subclasses to make a conversion.
		map<int,string> action_mapping_; 
		ros::ServiceClient pomdp_client_;  //pomdp library client
		ros::NodeHandle node_handle_;
		//utility method to call the appl pomdp library
		string updatePomdp(string command, string x_state, string obs);	
};

#endif