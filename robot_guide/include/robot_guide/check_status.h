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


class CheckStatus {
public:
	CheckStatus(ros::NodeHandle node_handle);
	bool isBatteryLow();
	bool isBumperPressed();



private:
	void bumperCallback(const spencer_control_msgs::SystemStatus& msg);
	void batteryCallback(const std_msgs::Float32& msg);

	ros::NodeHandle node_handle_;

	bool battery_low_;
	bool bumper_pressed_;

	//mutexs
	boost::mutex mutex_battery_low_;  
	boost::mutex mutex_bumper_pressed_;
};

#endif