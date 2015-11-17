/**
	observation_manager.h
	author: Michelangelo Fiore

	this class manages observations used by the pomdps (and potentially by the rest of the supervisions). It's goal is 
	receiving data (often numerical) and translating them to the pomdps format. 

	The system can be set to a simple_mode, where it will collect observations from people in a symbolic area linked to it,
	without considering if they are part of the group to follow (necessary if ids of passengers are not stable). 
	The robot will choose observations from the *best_agent*, i.e. the one moving in the direction of the robot and the closest.
*/


#ifndef OBSERVATION_MANAGER_H 
#define OBSERVATION_MANAGER_H

#include <ros/ros.h>
#include <vector>
#include <string>

#include <situation_assessment_msgs/FactList.h>
#include <situation_assessment_msgs/Fact.h>

//boost
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 
#include <boost/thread/condition_variable.hpp>


using namespace std;


//a small class that maps the observations linked to an agent. It's used only in the simple_mode at the moment.
class AgentObservation {
	public:
	string name;
	double distance;
	double delta_distance;
	string orientation;
	string is_moving;

};

class ObservationManager {
public:

	ObservationManager(ros::NodeHandle node_handle, string robot_name, string mode);

	//when we need to observe a particular group, we can set this parameter in the observation manager.
	void setObservedGroup(string group);
	void setAgentsInGroup(vector<string> agents_in_group);
	//the robot waits for the observed group to be present.
	void waitForGroup();
	//getters for the observations.
	string getDeltaDistance();
	string getGroupDistance();
	string getOrientation();
	string getGroupIsMoving();
	string getHighestDensity();
	string getInSlowArea();
	string getRobotLocation();

private:
	//callback to agent facts, as computed by our situation assessment modules
	void agentFactCallback(const situation_assessment_msgs::FactList::ConstPtr& msg);
	//pomdp observations and variables
	boost::mutex mutex_observations_;
	boost::mutex mutex_has_observed_group_;
	boost::condition_variable condition_has_observed_group_;
	boost::mutex mutex_agents_in_group;

	bool has_observed_group_;

	//parameters
	string robot_name_;
	string mode_;


	string observed_group_; //name of the group to observe


	vector<string> agents_in_group_;

	string best_agent_name_; //for simple mode, name of the agent currently tracked by the system


	//observations
	string delta_distance_;  //increasing, decreasing or stable
	string group_distance_;  //close, far, outOfRange
	string orientation_;     //towardRobot, other
	string group_is_moving_; //notMoving, moving, unknown
	string highest_density_; //sides, behind
	string in_slow_area_;    //false,true
	string robot_location_;  //location of the robot in the symbolic map

	string true_mode;

	//ros variables
	ros::NodeHandle node_handle_;
	ros::Subscriber agent_sub_;

	//function to get observation in the simple mode
	void getSimpleObservations(vector<situation_assessment_msgs::Fact> fact_list);
	void getComplexObservations(vector<situation_assessment_msgs::Fact> fact_list);
	AgentObservation getBestAgent(map<string,AgentObservation> agent_observations);
	void setSymbolicObservations(AgentObservation agent);
	map<string,AgentObservation> createAgentObservations(vector<situation_assessment_msgs::Fact> fact_list,vector<string> agents_to_find);

};

#endif