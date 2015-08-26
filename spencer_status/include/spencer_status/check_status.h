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

using namespace std;
class CheckStatus {
public:
	CheckStatus(ros::NodeHandle node_handle);
	bool isBatteryLow();
	bool isBumperPressed();
	bool isStopped();
	string getCommonErrorString();
private:
	void bumperCallback(const spencer_control_msgs::SystemStatus& msg);
	void batteryCallback(const std_msgs::Float32& msg);
	void stoppedCallback(const supervision_msgs::SupervisionStopped& msg);

	ros::NodeHandle node_handle_;

	bool battery_low_;
	bool bumper_pressed_;
	bool stopped_;

	//mutexs
	boost::mutex mutex_battery_low_;  
	boost::mutex mutex_bumper_pressed_;
	boost::mutex mutex_stopped_;

	ros::Subscriber bumper_sub,status_sub,battery_sub;
};

#endif