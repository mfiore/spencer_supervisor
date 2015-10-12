/**
	robot_status_simulator.cpp
	author: Michelangelo Fiore

	Simulates battery and emergency stop when not using the real robto.
*/

#include <ros/ros.h>
#include <string>
#include <spencer_control_msgs/SystemStatus.h>
#include <boost/thread/mutex.hpp>
#include <supervision_msgs/SetRobotSimStatus.h>
#include <std_msgs/Float32.h>

using namespace std;


bool software_emergency_stop=false;
bool hardware_emergency_stop=false;
int data=100;

boost::mutex mutex;

bool setRobotSimStatus(supervision_msgs::SetRobotSimStatus::Request &req, 
					   supervision_msgs::SetRobotSimStatus::Response &res) {
	mutex.lock();
	software_emergency_stop=req.software_emergency_stop;
	hardware_emergency_stop=req.hardware_emergency_stop;
	data=req.data;

	ROS_INFO("Setting new status");
	res.ok=true;

	mutex.unlock();
	return true;
}

int main(int argc, char ** argv) {
	ros::init(argc,argv,"robot_status_simulator");

	ros::NodeHandle node_handle;

	ros::Publisher status_publisher=node_handle.advertise<spencer_control_msgs::SystemStatus>("/spencer/control/system_status", 1000);
	ros::Publisher battery_publisher=node_handle.advertise<std_msgs::Float32>("/spencer/sensors/battery/percentage", 1000);


	ros::ServiceServer set_status_server=node_handle.advertiseService("/supervision/set_robot_sim_status",setRobotSimStatus);

	ros::Rate r(10);
	while (ros::ok()) {
		spencer_control_msgs::SystemStatus status_msg;
		std_msgs::Float32 battery_msg;

		mutex.lock();
		status_msg.software_emergency_stop=software_emergency_stop;
		status_msg.hardware_emergency_stop=hardware_emergency_stop;
		battery_msg.data=data;
		mutex.unlock();

		status_publisher.publish(status_msg);
		battery_publisher.publish(battery_msg);
		r.sleep();
	}
	ros::shutdown();
}

