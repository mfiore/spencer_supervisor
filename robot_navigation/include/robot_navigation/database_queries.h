#ifndef DATABASE_QUERIES_H
#define DATABASE_QUERIES_H

#include <ros/ros.h>
#include <situation_assessment_msgs/FactList.h>
#include <situation_assessment_msgs/QueryDatabase.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>


using namespace std;
class DatabaseQueries {
public:
	DatabaseQueries(ros::NodeHandle node_handle,string robot_name);
	geometry_msgs::Pose getPose(string name);
	bool hasArea(string name);
	string getEntityType(string name);
	string getRobotLocation(vector<string> robot_areas);
	string getEntityLocation(string name);
	geometry_msgs::Pose getRobotPose();
private:
	ros::NodeHandle node_handle_;
	ros::ServiceClient simple_database_client_;

	string robot_name_;

};

#endif