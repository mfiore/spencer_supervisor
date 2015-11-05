#include <robot_guide/observation_manager.h>

ObservationManager::ObservationManager(ros::NodeHandle node_handle, string robot_name, string mode):
node_handle_(node_handle),robot_name_(robot_name),mode_(mode) {
	ROS_INFO("OBSERVATION_MANAGER creating observation_manager");
	agent_sub_ = node_handle_.subscribe("/situation_assessment/agent_fact_list", 1000, 
		&ObservationManager::agentFactCallback,this);

	ros::Rate r(3);
	ROS_INFO("OBSERVATION_MANAGER Waiting for agent fact list to be published");
	while (agent_sub_.getNumPublishers()==0 && ros::ok()) {
		r.sleep();
	}
	ROS_INFO("OBSERVATION_MANAGER Done");

}

void ObservationManager::setObservedGroup(string group) {
	observed_group_=group;
}

//we use a condition variable to block until se receive observations of the group. This variable can be unlocked in the callbacks
void ObservationManager::waitForGroup() {
	ROS_INFO("OBSERVATION_MANAGER Waiting for group observations");
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


AgentObservation ObservationManager::getBestAgent(map<string,AgentObservation> agent_observations) {
	//now we find the best agent in this way:
	//best condition is the closet agent moving toward the robot.
	//orientation is the first parameter checked, than it's movement, delta_distance and finally distance.
	AgentObservation best_agent=agent_observations.begin()->second;
	for (map<string,AgentObservation>::iterator it=agent_observations.begin();it!=agent_observations.end();it++) {
		it->second.orientation="towardRobot";
		// ROS_INFO("OBSERVATION_MANAGER Comparing %s with %s",best_agent.name.c_str(),it->second.name.c_str());
		// ROS_INFO("OBSERVATION_MANAGER Best agent %s %s %f %f",best_agent.orientation.c_str(),best_agent.is_moving.c_str(),best_agent.delta_distance,best_agent.distance);
		// ROS_INFO("OBSERVATION_MANAGER Other agent %s %s %f %f",it->second.orientation.c_str(),it->second.is_moving.c_str(),it->second.delta_distance,it->second.distance);
		if (it->second.orientation=="towardRobot" && best_agent.orientation!="towardRobot") {
			// ROS_INFO("OBSERVATION_MANAGER Orinetation different");
			best_agent=it->second;
		}
		else if (it->second.orientation=="towardRobot" && it->second.is_moving=="moving" && best_agent.is_moving=="notMoving") {
			 // ROS_INFO("OBSERVATION_MANAGER One is moving and one not");
			best_agent=it->second;
		}
		else if (it->second.orientation=="towardRobot" && it->second.is_moving=="moving" && it->second.delta_distance>1 && best_agent.delta_distance<1){
			// ROS_INFO("OBSERVATION_MANAGER Delta distance different");
			best_agent=it->second;
		}
		else if (it->second.orientation=="towardRobot" && it->second.is_moving=="moving" && it->second.delta_distance>1 && 
			it->second.distance<best_agent.distance)
		 {
			// ROS_INFO("OBSERVATION_MANAGER Going for distance");
		 	best_agent=it->second;
		}
		else if (it->second.distance<best_agent.distance) {
			best_agent=it->second;
		}
	}
	return best_agent;
}


void ObservationManager::setSymbolicObservations(AgentObservation agent) {
	//now we map observations into symbolic strings used by the system
	if (agent.delta_distance<-1) {
		delta_distance_="increasing";
	}
	else if (agent.delta_distance>1) {
		delta_distance_="decreasing";
	}
	else {
		delta_distance_="stable";
	}


	if (agent.distance<3) {
		group_distance_="close";
	}
	else if (agent.distance<7) {
		group_distance_="far";
	}
	else group_distance_="outOfRange";


	if (agent.distance<1.5) {
		in_slow_area_="false";
		highest_density_="sides";
	}
	else if (agent.distance<3) {
		in_slow_area_="false";
		highest_density_="behind";
	}
	else {
		in_slow_area_="true";
		highest_density_="behind";
	}
	if (group_distance_!="outOfRange") {
		orientation_=agent.orientation;
	}
	else {
		orientation_="unknown";
	}
	// if (agent.is_moving=="moving") {
		// group_is_moving_="moving";
	// }
	// else {
		// group_is_moving_="notMoving";
	// }
	group_is_moving_=agent.is_moving;
}


map<string,AgentObservation> ObservationManager::createAgentObservations(vector<situation_assessment_msgs::Fact> fact_list,
	vector<string> agents_to_find) {
	//we get the observations linked to each agent calculated before. (since it's a map shouldn't be heavy)
	map<string,AgentObservation> agent_observations;
	BOOST_FOREACH(situation_assessment_msgs::Fact f, fact_list) {
		if (f.subject!=robot_name_) {
			if (f.predicate[0]=="distance" && f.predicate[1]==robot_name_) {
				if (std::find(agents_to_find.begin(),agents_to_find.end(),f.subject)!=agents_to_find.end()) {
					double distance_num=boost::lexical_cast<double>(f.value[0]);
					agent_observations[f.subject].distance=distance_num;
				}
			}
			else if (f.predicate[0]=="delta_distance" && f.predicate[1]==robot_name_) {
				if (std::find(agents_to_find.begin(),agents_to_find.end(),f.subject)!=agents_to_find.end()) {
					double delta_distance_num=boost::lexical_cast<double>(f.value[0]);
					agent_observations[f.subject].delta_distance=delta_distance_num;
				}
			}
			else if (f.predicate[0]=="orientation" && f.predicate[1]==robot_name_) {
				if (std::find(agents_to_find.begin(),agents_to_find.end(),f.subject)!=agents_to_find.end()) {
					agent_observations[f.subject].orientation=f.value[0];
				}
			}	
			else if (f.predicate[0]=="isMoving") {
				if (std::find(agents_to_find.begin(),agents_to_find.end(),f.subject)!=agents_to_find.end()) {
					if (f.value[0]=="0") {
						agent_observations[f.subject].is_moving="notMoving";
					}
					else {
						agent_observations[f.subject].is_moving="moving";
					}
				}
			}
		}
	}
	for (map<string,AgentObservation>::iterator it=agent_observations.begin();it!=agent_observations.end();it++) {
		it->second.name=it->first;
	}
	return agent_observations;

}


void ObservationManager::getSimpleObservations(vector<situation_assessment_msgs::Fact> fact_list) {

	//first, we get every agent in the robot area. Actually we might get objects too if we track their position in areas, but
	//at the moment we aren't so this works. If this changes, the procedure needs to read the list of agents.
	vector<string> agents,agents_to_find;

	BOOST_FOREACH(situation_assessment_msgs::Fact f, fact_list) {
		if (f.predicate[0]=="type" && f.value[0]=="agent" && f.value[1]=="HUMAN") {
				agents.push_back(f.subject);
		}

	}

	BOOST_FOREACH(situation_assessment_msgs::Fact f, fact_list) {
		if (f.subject!=robot_name_) {
			if (f.predicate[0]=="isInArea") {
				if (std::find(agents.begin(),agents.end(),f.subject)!=agents.end() 
					&& std::find(f.value.begin(),f.value.end(),robot_name_)!=f.value.end())  {
					agents_to_find.push_back(f.subject);
				}
			}
		}
	}	


	map<string,AgentObservation> agent_observations=createAgentObservations(fact_list,agents_to_find);
	if (agent_observations.size()>0) {
		AgentObservation best_agent=getBestAgent(agent_observations);
	
		if (best_agent_name_!=best_agent.name) {
			ROS_INFO("OBSERVATION_MANAGER Guiding agent %s",best_agent.name.c_str());
			best_agent_name_=best_agent.name;
		}
			setSymbolicObservations(best_agent);
		
				
		//and we unlock the condition variable. We're not guiding a particular group so anything found will be ok
		boost::lock_guard<boost::mutex> lock(mutex_has_observed_group_);
		if (has_observed_group_==false) {
			has_observed_group_=true;
			condition_has_observed_group_.notify_one();
		}
	}
	else { //if we didn't found people we set parameters to unknown
		orientation_="unknown";
		group_is_moving_="unknown";
		group_distance_="outOfRange";
		delta_distance_="unknown";
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

	//if we're in simple_mode we call the other procedure, else we proceed normally
	if (mode_=="simple") {
		getSimpleObservations(fact_list);
	}
 	else if (mode_=="complex"){
 		getComplexObservations(fact_list);
 	}
 	else {
 		//in the loop we collect observations of the group and of its members
		BOOST_FOREACH(situation_assessment_msgs::Fact f, fact_list) {
			if (f.subject==observed_group_ && f.predicate[0]=="contains") {
				agents_in_group=f.value;
			}
			if (f.subject==observed_group_ && f.predicate[0]=="delta_distance" && f.predicate[1]==robot_name_) {
				double delta_distance_num=boost::lexical_cast<double>(f.value[0]);
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

				double distance_num=boost::lexical_cast<double>(f.value[0]);
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
				double distance_num=boost::lexical_cast<double>(f.value[0]);
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
				if (f.value[0]=="0") {
					group_is_moving_="notMoving";
				}
				else {
					group_is_moving_="moving";
				}
			}
			else if (f.subject==robot_name_ && f.predicate[0]=="isInArea") {
				robot_location_=f.value[0];
			}

		}
	}
	if (mode_=="normal") {
		orientation_="towardRobot"; //again, this is a hack. If we get orientation from perception it's better
		if (num_sides>num_behind) {
			highest_density_="sides";
		}
		else {
			highest_density_="behind";
		}
	}

	//we unlock the condition variable if we found the group.
	boost::lock_guard<boost::mutex> lock(mutex_has_observed_group_);
	if (found_group) {
			has_observed_group_=true;
			condition_has_observed_group_.notify_one();
	}
}

void ObservationManager::getComplexObservations(vector<situation_assessment_msgs::Fact> fact_list) {

	mutex_agents_in_group.lock();


	vector<string> agents_to_find;
	BOOST_FOREACH(situation_assessment_msgs::Fact f, fact_list) {
		if (f.subject!=robot_name_) {
			if (f.predicate[0]=="isInArea") {
				if (std::find(agents_in_group_.begin(),agents_in_group_.end(),f.subject)!=agents_in_group_.end() && std::find(f.value.begin(),f.value.end(),robot_name_)!=f.value.end())  {
					agents_to_find.push_back(f.subject);
				}
			}
		}
	}	


	map<string,AgentObservation> agent_observations=createAgentObservations(fact_list,agents_to_find);
	// ROS_INFO("OBSERVATION_MANAGER ROBOT_GUIDE agent observations size %ld",agent_observations.size());
	if (agent_observations.size()>0) {
		if (true_mode!="complex") {
			ROS_INFO("OBSERVATION_MANAGER ROBOT_GUIDE using complex mode");
			true_mode="complex";
		}

		AgentObservation best_agent=getBestAgent(agent_observations);
		if (best_agent_name_!=best_agent.name) {
			ROS_INFO("OBSERVATION_MANAGER Guiding agent %s",best_agent.name.c_str());
			best_agent_name_=best_agent.name;
		}
		setSymbolicObservations(best_agent);
		boost::lock_guard<boost::mutex> lock(mutex_has_observed_group_);
		if (has_observed_group_==false) {
			has_observed_group_=true;
			condition_has_observed_group_.notify_one();
		}
	}
	else {
		if (true_mode!="simple") {
		ROS_INFO("OBSERVATION_MANAGER ROBOT_GUIDE no agent of group around. Switching to simple_mode");
		true_mode="simple";
		}
		getSimpleObservations(fact_list);
	}
	mutex_agents_in_group.unlock();
}

void ObservationManager::setAgentsInGroup(vector<string> agents_in_group) {
	boost::lock_guard<boost::mutex> lock(mutex_agents_in_group);
	agents_in_group_=agents_in_group;
	ROS_INFO("OBSERVATION_MANAGER ROBOT_GUIDE agents in group are:");
	for (int i=0; i<agents_in_group.size();i++) {
		ROS_INFO("OBSERVATION_MANAGER ROBOT_GUIDE %s",agents_in_group[i].c_str());
	}
}

