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

class ObservationManager {
public:
	ObservationManager(ros::NodeHandle node_handle, string robot_name);

	void setObservedGroup(string group);
	void waitForGroup();
	string getDeltaDistance();
	string getGroupDistance();
	string getOrientation();
	string getGroupIsMoving();
	string getHighestDensity();
	string getInSlowArea();
	string getRobotLocation();
	string getTimer();

private:
	void agentFactCallback(const situation_assessment_msgs::FactList::ConstPtr& msg);
	//pomdp observations and variables
	boost::mutex mutex_observations_;
	boost::mutex mutex_has_observed_group_;
	boost::condition_variable condition_has_observed_group_;
	bool has_observed_group_;


	string robot_name_;
	string observed_group_;

	string delta_distance_;
	string group_distance_;
	string orientation_;
	string group_is_moving_;
	string highest_density_;
	string in_slow_area_;
	string robot_location_;

	ros::NodeHandle node_handle_;

	ros::Subscriber agent_sub_;
};