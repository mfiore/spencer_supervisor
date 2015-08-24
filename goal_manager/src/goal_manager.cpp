#include <ros/ros.h>

#include <string>
#include <vector>

#include <supervision_msgs/ApproachAction.h>
#include <supervision_msgs/GuideGroupAction.h>
#include <supervision_msgs/MoveToAction.h>
#include <supervision_msgs/MoveToPoseAction.h>

typedef actionlib::SimpleActionServer<supervision_msgs::GuideGroupAction> GuideServer;
typedef actionlib::SimpleActionServer<supervision_msgs::MoveToAction> MoveToServer;
typedef actionlib::SimpleActionServer<supervision_msgs::ApproachAction> ApproachServer;

typedef actionlib::SimpleActionClient<supervision_msgs::GuideGroupAction> GuideClient;
typedef actionlib::SimpleActionClient<supervision_msgs::MoveToAction> MoveToClient;
typedef actionlib::SimpleActionClient<supervision_msgs::MoveToPoseAction> MoveToPoseClient;
typedef actionlib::SimpleActionClient<supervision_msgs::ApproachAction> ApproachClient;


int main(int argc, char** argv) {
	ros::init(argc,arg,"goal_manager");
	ros::NodeHandle node_handle;


}