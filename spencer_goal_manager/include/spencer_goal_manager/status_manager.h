#ifndef STATUS_MANAGER_H
#define STATUS_MANAGER_H

#include <ros/ros.h>

#include <string>
#include <vector>
#include <supervision_msgs/SupervisionStatus.h>
#include <situation_assessment_msgs/QueryDatabase.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 

using namespace std;

class StatusManager {
public:
	StatusManager(ros::NodeHandle node_handle);

	string getLocation();
	string getStatus();
private:
	void guideStatusCallback(const supervision_msgs::SupervisionStatus::ConstPtr& msg);
	void moveStatusCallback(const supervision_msgs::SupervisionStatus::ConstPtr& msg);

	string guide_status_;
	string move_status_;
	ros::NodeHandle node_handle_;

	ros::Subscriber guide_status_sub_,move_status_sub_;
	ros::ServiceClient database_service_;

	string robot_name_;

	boost::mutex mutex_guide_status_, mutex_move_status_;

};

#endif
