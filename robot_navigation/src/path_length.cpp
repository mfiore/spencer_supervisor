
#include <robot_navigation/path_length.h>

PathLength::PathLength(ros::NodeHandle node_handle,DatabaseQueries *database_queries):node_handle_(node_handle),
database_queries_(database_queries) {
	path_info_publisher_=node_handle.advertise<supervision_msgs::PathInfo>("/supervision/path_info",1000);
	robot_speed_=0.7;
}

void PathLength::startPublishingPath(vector<geometry_msgs::Pose> path, double previous_length) {
	should_publish_=true;
	current_node_=0;
	double tot_length=0;
	
	geometry_msgs::Pose robot_pose=database_queries_->getRobotPose();
	double distance_to_next=dist2d(robot_pose,path[1]);
	
	if (previous_length==-1){
	   tot_length=calculateRemainingLength(path);
	   tot_length=tot_length+distance_to_next;
	}
	else {
		tot_length=previous_length;
	}

	int tot_seconds=round(tot_length/robot_speed_);
	setTotalLength(tot_length);


	int path_index=-1;
	double remaining_path_length;	
	ros::Rate r(1);
	while (shouldPublish()) {
		supervision_msgs::PathInfo msg;


		int new_index=getCurrentNode();
		if (new_index<path.size()-1) {
			robot_pose=database_queries_->getRobotPose();
			distance_to_next=dist2d(robot_pose,path[new_index+1]);
		}
		if (new_index!=path_index) {
			path_index=new_index;
			remaining_path_length=calculateRemainingLength(path);
			ROS_INFO("Remaining length is %f",remaining_path_length);
		}
		double remaining_length=remaining_path_length+distance_to_next;
		int remaining_seconds=round(remaining_length/robot_speed_);

		msg.total_length=tot_length;
		msg.total_seconds=tot_seconds-2;
		msg.remaining_length=remaining_length;
		msg.remaining_seconds=remaining_seconds-2;

		path_info_publisher_.publish(msg);
		r.sleep();
	}
}
void PathLength::stopPublishingPath() {
	boost::lock_guard<boost::mutex> lock(mutex_should_publish_);
	should_publish_=false;
}

bool PathLength::shouldPublish() {
	boost::lock_guard<boost::mutex> lock(mutex_should_publish_);
	return should_publish_;

}


void PathLength::updateCurrentNode(int i) {
	boost::lock_guard<boost::mutex> lock(mutex_current_node_);
	current_node_=i;
}
int PathLength::getCurrentNode() {
	boost::lock_guard<boost::mutex> lock(mutex_current_node_);
	return current_node_;
}

double PathLength::calculatePathLength(vector<geometry_msgs::Pose> path) {
	double tot=0;

	if (path.size()==1) return 0;
	for (int i=0; i<path.size()-1;i++) {
		tot=tot+dist2d(path[i],path[i+1]);
	}
	return tot;
}

double PathLength::dist2d(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
	double x1,x2,y1,y2;
	x1=p1.position.x;
	x2=p2.position.x;
	y1=p1.position.y;
	y2=p2.position.y;
	return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}

double PathLength::calculateRemainingLength(vector<geometry_msgs::Pose> path) {
	int i=getCurrentNode();	
	if (i==path.size()-1) return 0;
	double tot=0;
	if (i>=path.size()) {
		ROS_WARN("ROBOT_NAVIGATION path index bigger than path size");
		return 0;
	}
	i++;
	while (i<path.size()-1) {
		tot=tot+dist2d(path[i],path[i+1]);
		i++;
	}
	return tot;
}

double PathLength::getTotalLength() {
	boost::lock_guard<boost::mutex> lock(mutex_length_);
	return total_length_;
}
void PathLength::setTotalLength(double length) {
	boost::lock_guard<boost::mutex> lock(mutex_length_);
	ROS_INFO("Setting total length as %f",length);
	total_length_=length;
}