/**
	check_status.cpp
	author: Michelangelo Fiore

	This class contains procedurs to monitor the status of several robot components.

*/

#include <spencer_status/check_status.h>

CheckStatus::CheckStatus(ros::NodeHandle node_handle):node_handle_(node_handle) {

	bumper_sub = node_handle_.subscribe("/spencer/control/system_status", 1000, 
		&CheckStatus::bumperCallback, this);
	ros::Rate r(10);
	ROS_INFO("Waiting for system status to be published");
	while (bumper_sub.getNumPublishers()==0 && ros::ok()) {
		r.sleep();
	}

	battery_sub = node_handle_.subscribe("/spencer/sensors/battery/percentage", 1000, 
		&CheckStatus::batteryCallback,this);
	ROS_INFO("Waiting for battery status to be published");
	while (battery_sub.getNumPublishers()==0  && ros::ok()) {
		r.sleep();

	}	

	status_sub = node_handle_.subscribe("/supervision/is_stopped", 1000, 
		&CheckStatus::stoppedCallback,this);

	battery_low_=false;
	bumper_pressed_=false;
	stopped_=false;
}

void CheckStatus::bumperCallback(const spencer_control_msgs::SystemStatus& msg) {
	boost::lock_guard<boost::mutex> guard(mutex_bumper_pressed_);
	if (msg.software_emergency_stop || msg.hardware_emergency_stop) {
		if (bumper_pressed_==false) {
			bumper_pressed_=true;
			ROS_INFO("Emergency stop");
		}
	}
	else {
		bumper_pressed_=false;
	}
}
void CheckStatus::batteryCallback(const std_msgs::Float32& msg) {
	boost::lock_guard<boost::mutex> guard(mutex_battery_low_);
	if (msg.data<10) {
		if (battery_low_==false) {
			battery_low_=true;
			ROS_INFO("Battery level too low");
		}
	}
	else {
		battery_low_=false;
	}
}

void CheckStatus::stoppedCallback(const supervision_msgs::SupervisionStopped& msg) {
	boost::lock_guard<boost::mutex> guard(mutex_stopped_);
	stopped_=msg.is_stopped;
}


bool CheckStatus::isBatteryLow() {
	boost::lock_guard<boost::mutex> guard(mutex_battery_low_);
	return battery_low_;
}
bool CheckStatus::isBumperPressed() {
	boost::lock_guard<boost::mutex> guard(mutex_bumper_pressed_);
	return bumper_pressed_;
}

bool CheckStatus::isStopped() {
	boost::lock_guard<boost::mutex> guard(mutex_stopped_);
	return stopped_;
}

string CheckStatus::getCommonErrorString() {

	if (!ros::ok()) {
		return "node shutdown";
	}
	if (isBumperPressed()) {
		return "Bumper pressed or Emergency Stop";
	}
	else if (isBatteryLow()) {
		return "Battery level is low";
	}
	else if (isStopped()) {
		return "Supervisor stopped";
	}
	else {
		return "Other Error";
	}
	

}