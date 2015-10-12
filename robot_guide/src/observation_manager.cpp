#include <robot_guide/observation_manager.h>

ObservationManager::ObservationManager(ros::NodeHandle node_handle, string robot_name, bool simple_mode):
node_handle_(node_handle),robot_name_(robot_name),simple_mode_(simple_mode) {
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


void ObservationManager::getSimpleObservations(vector<situation_assessment_msgs::Fact> fact_list) {


	map<string,AgentObservation> agent_observations;
	BOOST_FOREACH(situation_assessment_msgs::Fact f, fact_list) {
		if (f.subject!=robot_name_) {
			if (f.predicate[0]=="isInArea" && f.value==robot_name_) {
				AgentObservation new_agent;
				new_agent.name=f.subject;
				agent_observations[new_agent.name]=new_agent;
			}
		}
	}
	BOOST_FOREACH(situation_assessment_msgs::Fact f, fact_list) {
		if (f.subject!=robot_name_) {
			if (f.predicate[0]=="distance" && f.predicate[1]==robot_name_) {
				if (agent_observations.find(f.subject)!=agent_observations.end()) {
					double distance_num=boost::lexical_cast<double>(f.value);
					agent_observations[f.subject].distance=distance_num;
				}
			}
			else if (f.predicate[0]=="delta_distance" && f.predicate[1]==robot_name_) {
				if (agent_observations.find(f.subject)!=agent_observations.end()) {
					double delta_distance_num=boost::lexical_cast<double>(f.value);
					agent_observations[f.subject].delta_distance=delta_distance_num;
				}
			}
			else if (f.predicate[0]=="orientation" && f.predicate[1]==robot_name_) {
				if (agent_observations.find(f.subject)!=agent_observations.end()) {
					agent_observations[f.subject].orientation=f.value;
				}
			}	
			else if (f.predicate[0]=="isMoving") {
				if (agent_observations.find(f.subject)!=agent_observations.end()) {
					if (f.value=="0") {
						agent_observations[f.subject].is_moving="notMoving";
					}
					else {
						agent_observations[f.subject].is_moving="moving";
					}
				}
			}
		}
	}
	if (agent_observations.size()>0) {
		AgentObservation best_agent=agent_observations.begin()->second;
		best_agent.orientation="towardRobot";
		// ROS_INFO("Start check");
		for (map<string,AgentObservation>::iterator it=agent_observations.begin();it!=agent_observations.end();it++) {
			it->second.orientation="towardRobot";
			// ROS_INFO("Comparing %s with %s",best_agent.name.c_str(),it->second.name.c_str());
			// ROS_INFO("Best agent %s %s %f %f",best_agent.orientation.c_str(),best_agent.is_moving.c_str(),best_agent.delta_distance,best_agent.distance);
			// ROS_INFO("Other agent %s %s %f %f",it->second.orientation.c_str(),it->second.is_moving.c_str(),it->second.delta_distance,it->second.distance);
			if (it->second.orientation=="towardRobot" && best_agent.orientation!="towardRobot") {
				// ROS_INFO("Orinetation different");
				best_agent=it->second;
			}
			else if (it->second.orientation=="towardRobot" && it->second.is_moving=="moving" && best_agent.is_moving=="notMoving") {
				 ROS_INFO("One is moving and one not");
				best_agent=it->second;
			}
			else if (it->second.orientation=="towardRobot" && it->second.is_moving=="moving" && it->second.delta_distance>1 && best_agent.delta_distance<1){
				// ROS_INFO("Delta distance different");
				best_agent=it->second;
			}
			else if (it->second.orientation=="towardRobot" && it->second.is_moving=="moving" && it->second.delta_distance>1 && 
				it->second.distance<best_agent.distance)
			 {
				// ROS_INFO("Going for distance");
			 	best_agent=it->second;
			}
			else if (it->second.distance<best_agent.distance) {
				best_agent=it->second;
			}
		}
		if (best_agent_name_!=best_agent.name) {
			ROS_INFO("Guiding agent %s",best_agent.name.c_str());
			best_agent_name_=best_agent.name;
		}

		if (best_agent.delta_distance<-1) {
			delta_distance_="increasing";
		}
		else if (best_agent.delta_distance>1) {
			delta_distance_="decreasing";
		}
		else {
			delta_distance_="stable";
		}

		if (best_agent.distance<4) {
			group_distance_="close";
		}
		else if (best_agent.distance<10) {
			group_distance_="far";
		}
		else group_distance_="outOfRange";


		if (best_agent.distance<1.5) {
			in_slow_area_="false";
			highest_density_="sides";
		}
		else if (best_agent.distance<3) {
			in_slow_area_="false";
			highest_density_="behind";
		}
		else {
			in_slow_area_="true";
			highest_density_="behind";
		}
		orientation_="towardRobot";
		group_is_moving_=best_agent.is_moving;
			
		


		boost::lock_guard<boost::mutex> lock(mutex_has_observed_group_);
		if (has_observed_group_==false) {
			has_observed_group_=true;
			condition_has_observed_group_.notify_one();
		}
	}
	else {
		orientation_="unknown";
		group_is_moving_="unknown";
		group_distance_="outOfRange";
	}
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


	if (simple_mode_) {
		getSimpleObservations(fact_list);
	}
 	else {
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

				if (agents_in_group.size()>0) {
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
	}
	if (!simple_mode_) {
		orientation_="towardRobot";
		if (num_sides>num_behind) {
			highest_density_="sides";
		}
		else {
			highest_density_="behind";
		}
	}

	boost::lock_guard<boost::mutex> lock(mutex_has_observed_group_);
	if (found_group) {
			has_observed_group_=true;
			condition_has_observed_group_.notify_one();
	}
}