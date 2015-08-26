#include <ros/ros.h>
#include <supervision_msgs/EmptyRequest.h>
#include <boost/thread/mutex.hpp>
#include <supervision_msgs/SupervisionStopped.h>

ros::Publisher status_pub;

boost::mutex stopped_mutex;

bool stop_supervision;

//stops the supervision system
bool stopSupervision(supervision_msgs::EmptyRequest::Request &req, supervision_msgs::EmptyRequest::Response &res) {
	stopped_mutex.lock();
	stop_supervision=true;
	stopped_mutex.unlock();

	res.result="OK";
	return true;
}

//restarts the supervision system
bool restartSupervision(supervision_msgs::EmptyRequest::Request &req, supervision_msgs::EmptyRequest::Response &res) {
	stopped_mutex.lock();
	stop_supervision=false;
	stopped_mutex.unlock();

	return true;
}



int main(int argc, char** argv) {
	ros::init(argc,argv,"spencer_status");
	ros::NodeHandle n;

	ROS_INFO("Started spencer_status");

	//advertise servers, topics and actions
	ros::ServiceServer stop_superivision_server=n.advertiseService("supervision/stop",stopSupervision);
	ros::ServiceServer restart_superivision_server=n.advertiseService("supervision/restart",restartSupervision);

	status_pub=n.advertise<supervision_msgs::SupervisionStopped>("supervision/is_stopped",1000);
	
	while (ros::ok()) {
		ros::spinOnce();

		stopped_mutex.lock();
		supervision_msgs::SupervisionStopped msg;
		msg.is_stopped=stop_supervision;
		status_pub.publish(msg);
	}

}