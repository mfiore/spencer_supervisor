#ifndef PATH_LENGHT_H
#define PATH_LENGHT_H

#include <ros/ros.h>
#include <supervision_msgs/PathInfo.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <vector>
#include <string>
#include <robot_navigation/database_queries.h>
#include <geometry_msgs/Pose.h>

using namespace std;

class PathLength {
public:
	PathLength(ros::NodeHandle node_handle, DatabaseQueries *database_queries);
	void startPublishingPath(vector<geometry_msgs::Pose> path,double previous_length);
	void stopPublishingPath();
	void updateCurrentNode(int i);
	bool shouldPublish();
	int getCurrentNode();
	geometry_msgs::Pose getRobotLocation();
	double getTotalLength();


private:
	double calculatePathLength(vector<geometry_msgs::Pose> path) ;
	double dist2d(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
	double calculateRemainingLength(vector<geometry_msgs::Pose> path);

	void setTotalLength(double length);
	boost::mutex mutex_length_;
	double total_length_;

	bool should_publish_;
	boost::mutex mutex_should_publish_;


	ros::NodeHandle node_handle_;
	ros::Publisher path_info_publisher_;

	double robot_speed_;

	DatabaseQueries *database_queries_;


	boost::mutex mutex_current_node_;
	int current_node_;
};

#endif