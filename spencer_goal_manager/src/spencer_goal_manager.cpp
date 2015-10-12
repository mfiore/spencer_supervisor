#include <ros/ros.h>

#include <string>
#include <vector>
#include <supervision_msgs/GuideGroupAction.h>
#include <supervision_msgs/MoveToAction.h>
#include <supervision_msgs/MoveToAction.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 


boost::condition_variable 

using namespace std;

typedef actionlib::SimpleActionClient<supervision_msgs::GuideGroupAction> GuideGroupClient; 
typedef actionlib::SimpleActionClient<supervision_msgs::MoveToAction> MoveToClient; 

GuideGroupClient* guide_group_client;
MoveToClient* move_to_client;

bool is_guiding;
boost::mutex mutex_guide_group;
boost::condition_variable guide_group_condition;

//publishers
ros::Publisher status_pub;

void changeGate(string new_gate) {
	supervision_msgs::GuideGroupGoal guide_goal;

	guide_goal.destination=new_gate;

	supervision_msgs::SupervisionStatus status_msg; 
	status_msg.status="Changing gate to "+new_gate;
	status_pub.publish(status_msg);

	if (guide_group_client->getState()==actionlib::SimpleClientGoalState::ACTIVE) {
		guide_group_client->cancelGoal();
	}
	guide_group_client->cancelGoal();
	
	boost::unique_lock<boost::mutex> lock(mutex_guide_group);
	while (is_guiding) {
		guide_group_condition.wait(lock);
	}
	lock.unlock()




	guideGroup(new_gate);
}

void guideGroup(string gate) {
	supervision_msgs::GuideGroupGoal guide_goal;

	supervision_msgs::SupervisionStatus status_msg; 
	status_msg.status="Guiding group to "+gate;
	status_pub.publish(status_msg);

	guide_goal.destination=gate;	

	guide_group_client->sendGoal(guide_goal);
	is_guiding=true;
	guide_group_client->waitForResult();
	is_guiding=false;

	boost::unique_lock<boost::mutex> lock(mutex_guide_group);
	is_guiding=false;
	guide_group_condition.notify_one():
}


int main(int argc, char** argv) {
	ros::init(argc,argv,"spencer_goal_manager");

	guide_group_client=new GuideGroupClient("supervision/guide_group",true);
	move_to_client=new MoveToClient("supervision/move_to",true);

	guide_group_client.waitForServer();
	move_to_client.waitForServer();

	status_pub=n.advertise<supervision_msgs::SupervisionStatus>("supervision/goal",1000);


	return 0;
}