#include <robot_navigation/path_publisher.h>

PathPublisher::PathPublisher(ros::NodeHandle n):node_handle_(n) {
	path_publisher_=node_handle_.advertise<nav_msgs::Path>("supervision/navigation/path",1000);
}

void PathPublisher::publishPath(vector<geometry_msgs::Pose> node_poses,bool no_locations) {

	setShouldPublish(true);
	ros::Rate r(3);

	nav_msgs::Path path_msg;
	path_msg.header.frame_id="map";
	path_msg.header.stamp=ros::Time::now();
	for (int i=0;i<node_poses.size();i++) {
		if (i==0 && no_locations) continue;
		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header.frame_id="map";
		pose_stamped.header.stamp=ros::Time::now();
		pose_stamped.pose=node_poses[i];
		path_msg.poses.push_back(pose_stamped);
	}
	while (shouldPublish()) {
		path_publisher_.publish(path_msg);
		r.sleep();
	}
}


bool PathPublisher::shouldPublish() {
	boost::lock_guard<boost::mutex> lock(mutex_should_publish_);
	return should_publish_;
}
void PathPublisher::setShouldPublish(bool flag) {
	boost::lock_guard<boost::mutex> lock(mutex_should_publish_);
	should_publish_=flag;
}