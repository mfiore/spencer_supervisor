#include <spencer_goal_manager/status_manager.h>


StatusManager::StatusManager(ros::NodeHandle node_handle):node_handle_(node_handle) {
	guide_status_sub_=node_handle_.subscribe("/supervision/robot_guide/status",1,&StatusManager::guideStatusCallback,this);
	move_status_sub_=node_handle_.subscribe("/supervision/navigation/status",1,&StatusManager::moveStatusCallback,this);

	ROS_INFO("Waiting for status to be published");
	ros::Rate r(3);
	while (move_status_sub_.getNumPublishers()==0 && ros::ok()) {
		r.sleep();
	}
	ROS_INFO("Published");

	database_service_=node_handle_.serviceClient<situation_assessment_msgs::QueryDatabase>("situation_assessment/query_database");
	ROS_INFO("SPENCER_GOAL_MANAGER waiting for query database service");
	database_service_.waitForExistence();

	node_handle_.getParam("/robot/name",robot_name_);

	move_status_="IDLE";
}

string StatusManager::getLocation() {
	situation_assessment_msgs::QueryDatabase srv;
	srv.request.query.model=robot_name_;
	srv.request.query.subject=robot_name_;
	srv.request.query.predicate.push_back("isInArea");

	if (database_service_.call(srv)) {
		vector<string> robot_areas=srv.response.result[0].value;
		
		for (int i=0; i<robot_areas.size();i++) {
			srv.request.query.predicate.clear();
			srv.request.query.subject=robot_areas[i];
			srv.request.query.predicate.push_back("type");

			if (database_service_.call(srv)) {
				string type=srv.response.result[0].value[0];
				if (type=="location") return type;
			}
			else {
				ROS_WARN("SPENCER_GOAL_MANAGER couldn't contact database");
				return "";
			}
		}
	}
	else {
		ROS_WARN("SPENCER_GOAL_MANAGER couldn't contact database");
		return "";
	}
	ROS_WARN("SPENCER_GOAL_MANAGER couldn't find location");
	return "";
}

string StatusManager::getStatus(){
	boost::lock_guard<boost::mutex> lock(mutex_guide_status_);
	boost::lock_guard<boost::mutex> lock2(mutex_move_status_);
	if (guide_status_!="") {
		// ROS_INFO("Returning %s",guide_status_.c_str());
		return guide_status_;
	}
	else{
		// ROS_INFO("Returning %s",move_status_.c_str());
		return move_status_;	
	} 
}

void StatusManager::guideStatusCallback(const supervision_msgs::SupervisionStatus::ConstPtr& msg) {
	boost::lock_guard<boost::mutex> lock(mutex_guide_status_);
	guide_status_=msg->status+" "+msg->details;
	// ROS_INFO("Received status %s",guide_status_.c_str());
}
void StatusManager::moveStatusCallback(const supervision_msgs::SupervisionStatus::ConstPtr& msg) {
	boost::lock_guard<boost::mutex> lock(mutex_move_status_);
	move_status_=msg->status+" "+msg->details;
	// ROS_INFO("Received status %s",move_status_.c_str());
}

