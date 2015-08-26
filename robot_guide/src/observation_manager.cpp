#include <robot_guide/observation_manager.h>

ObservationManager::ObservationManager(ros::NodeHandle node_handle, string robot_name):
node_handle_(node_handle),robot_name_(robot_name) {
	ROS_INFO("creating observation_manager");
	agent_sub_ = node_handle_.subscribe("/situation_assessment/agent_fact_list", 1000, 
		&ObservationManager::agentFactCallback,this);

	ros::Rate r(3);
	ROS_INFO("Waiting for agent fact list to be published");
	while (agent_sub_.getNumPublishers()==0 && ros::ok()) {
		r.sleep();
	}
	ROS_INFO("Done");

}

void ObservationManager::setObservedGroup(string group) {
	observed_group_=group;
}

void ObservationManager::waitForGroup() {
	ROS_INFO("Waiting for group observations");
	boost::unique_lock<boost::mutex> lock(mutex_has_observed_group_);
	while (!has_observed_group_ && ros::ok()) {
		condition_has_observed_group_.wait(lock);
	}
	lock.unlock();
}
string ObservationManager::getDeltaDistance() {
	boost::lock_guard<boost::mutex> lock(mutex_observations_);
	return delta_distance_;
}
string ObservationManager::getGroupDistance() {
	boost::lock_guard<boost::mutex> lock(mutex_observations_);
	return group_distance_;
}
string ObservationManager::getOrientation() {
	boost::lock_guard<boost::mutex> lock(mutex_observations_);
	return orientation_;
}
string ObservationManager::getGroupIsMoving(){
	boost::lock_guard<boost::mutex> lock(mutex_observations_);
	return group_is_moving_;	
}
string ObservationManager::getHighestDensity() {
	boost::lock_guard<boost::mutex> lock(mutex_observations_);
	return highest_density_;
}
string ObservationManager::getInSlowArea() {
	boost::lock_guard<boost::mutex> lock(mutex_observations_);
	return in_slow_area_;
}
string ObservationManager::getRobotLocation() {
	boost::lock_guard<boost::mutex> lock(mutex_observations_);
	return robot_location_;
}

string ObservationManager::getTimer() {
	return "ok";
}


//Collect observations for the task
void ObservationManager::agentFactCallback(const situation_assessment_msgs::FactList::ConstPtr& msg) {
	vector<situation_assessment_msgs::Fact> fact_list=msg->fact_list;

	int num_sides=0;
	int num_behind=0;

	bool found_group=false;

	boost::lock_guard<boost::mutex> guard(mutex_observations_);


	ros::Rate r(3);
	vector<string> agents_in_group;
	BOOST_FOREACH(situation_assessment_msgs::Fact f, fact_list) {
		if (f.subject==observed_group_ && f.predicate[0]=="contains") {
			agents_in_group.push_back(f.value);
		}
		if (f.subject==observed_group_ && f.predicate[0]=="delta_distance" && f.predicate[1]==robot_name_) {
			double delta_distance_num=boost::lexical_cast<double>(f.value);
			if (delta_distance_num<-1) {
				delta_distance_="increasing";
			}
			else if (delta_distance_num>1) {
				delta_distance_="decreasing";
			}
			else {
				delta_distance_="stable";
			}
		}
		else if (f.subject==observed_group_ && f.predicate[0]=="distance" && f.predicate[1]==robot_name_) {
			found_group=true;

			double distance_num=boost::lexical_cast<double>(f.value);
			if (distance_num<4) {
				group_distance_="close";
			}
			else if (distance_num<10) {
				group_distance_="far";
			}
			else group_distance_="outOfRange";

			if (observed_group_==agents_in_group[0]) {
				if (distance_num<1.5) {
					num_sides++;
					in_slow_area_="false";
				}
				else if (distance_num<3) {
					num_behind++;
					in_slow_area_="false";
				}
				else {
					in_slow_area_="true";
				}
			}
		}
		else if (std::find(agents_in_group.begin(),agents_in_group.end(),f.subject)!=agents_in_group.end() &&
				 f.predicate[0]=="distance" && f.predicate[1]==robot_name_) {
			double distance_num=boost::lexical_cast<double>(f.value);
			if (distance_num<1.5) {
				num_sides++;
				in_slow_area_="false";
			}
			else if (distance_num<3) {
				num_behind++;
				in_slow_area_="false";
			}
			else {
				in_slow_area_="true";
			}
		}
		else if (f.subject==observed_group_ && f.predicate[0]=="isMoving") {
			if (f.value=="0") {
				group_is_moving_="notMoving";
			}
			else {
				group_is_moving_="moving";
			}
		}
		else if (f.subject==robot_name_ && f.predicate[0]=="isInArea") {
			robot_location_=f.value;
		}


	}
	orientation_="towardRobot";
	if (num_sides>num_behind) {
		highest_density_="sides";
	}
	else {
		highest_density_="behind";
	}

	boost::lock_guard<boost::mutex> lock(mutex_has_observed_group_);
	if (found_group) {
			has_observed_group_=true;
			condition_has_observed_group_.notify_one();
	}
}