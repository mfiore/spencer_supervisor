#ifndef PATH_PUBLISHER
#define PATH_PUBLISHER

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <vector>
#include <string>

using namespace std;

class PathPublisher {
public:
	PathPublisher(ros::NodeHandle n);

	bool shouldPublish();
	void setShouldPublish(bool flag);
	void publishPath(vector<geometry_msgs::Pose> path, bool no_locations);

private:
	ros::Publisher path_publisher_;
	bool should_publish_;
	boost::mutex mutex_should_publish_;

	ros::NodeHandle node_handle_;
};


#endif