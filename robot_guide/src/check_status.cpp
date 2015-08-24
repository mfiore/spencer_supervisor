/**
	check_status.cpp
	author: Michelangelo Fiore

	This class contains procedurs to monitor the status of several robot components.

*/

#include <robot_guide/check_status.h>

CheckStatus::CheckStatus(ros::NodeHandle node_handle):node_handle_(node_handle) {

	ros::Subscriber bumper_sub = node_handle_.subscribe("/spencer/control/system_status", 1000, 
		&CheckStatus::bumperCallback, this);
	ros::Rate r(10);
	ROS_INFO("Waiting for system status to be published");
	while (bumper_sub.getNumPublishers()==0 && ros::ok()) {
		r.sleep();
	}

	ros::Subscriber battery_sub = node_handle_.subscribe("/spencer/sensors/battery/percentage", 1000, 
		&CheckStatus::batteryCallback,this);
	ROS_INFO("Waiting for battery status to be published");
	while (battery_sub.getNumPublishers()==0  && ros::ok()) {
		r.sleep();

	}


	battery_low_=false;
	bumper_pressed_=false;
}

void CheckStatus::bumperCallback(const spencer_control_msgs::SystemStatus& msg) {
	boost::lock_guard<boost::mutex> guard(mutex_bumper_pressed_);
	if (msg.software_emergency_stop || msg.hardware_emergency_stop) {
		bumper_pressed_=true;
		ROS_INFO("Emergency stop");
	}
	else {
		bumper_pressed_=false;
	}
}
void CheckStatus::batteryCallback(const std_msgs::Float32& msg) {
	boost::lock_guard<boost::mutex> guard(mutex_battery_low_);
	if (msg.data<10) {
		battery_low_=true;
		ROS_INFO("Battery level too low");
	}
	else {
		battery_low_=false;
	}
}


bool CheckStatus::isBatteryLow() {
	boost::lock_guard<boost::mutex> guard(mutex_battery_low_);
	return battery_low_;
}
bool CheckStatus::isBumperPressed() {
	boost::lock_guard<boost::mutex> guard(mutex_bumper_pressed_);
	return bumper_pressed_;
}
