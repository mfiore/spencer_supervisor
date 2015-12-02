/**
	check_status.h
	author: Michelangelo Fiore

	This class contains procedurs to monitor the status of several robot components.

*/

#ifndef CHECK_STATUS_H
#define CHECK_STATUS_H

#include <ros/ros.h>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 
//#include <boost/thread/lock_guard.hpp> 
#include <spencer_control_msgs/SystemStatus.h>
#include <std_msgs/Float32.h>
#include <supervision_msgs/SupervisionStopped.h>
#include <std_msgs/Bool.h>

using namespace std;
class CheckStatus {
public:
	CheckStatus(ros::NodeHandle node_handle);
	bool isBatteryLow();
	bool isBumperPressed();
	bool isStopped();
	bool isPaused();
	bool isPlannerBlocked();
	string getCommonErrorString();
private:
	void bumperCallback(const spencer_control_msgs::SystemStatus& msg);
	void batteryCallback(const std_msgs::Float32& msg);
	void stoppedCallback(const supervision_msgs::SupervisionStopped& msg);
	// void plannerBlockedCallback(const std_msgs::Bool& msg);

	ros::NodeHandle node_handle_;

	bool battery_low_;
	bool bumper_pressed_;
	bool stopped_;
	bool paused_;
	bool planner_blocked_;

	//mutexs
	boost::mutex mutex_battery_low_;  
	boost::mutex mutex_bumper_pressed_;
	boost::mutex mutex_stopped_;
	boost::mutex mutex_planner_blocked_;

	ros::Subscriber bumper_sub,status_sub,battery_sub, planner_blocked_sub_;
};

#endif