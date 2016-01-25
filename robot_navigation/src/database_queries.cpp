#include <robot_navigation/database_queries.h>

DatabaseQueries::DatabaseQueries(ros::NodeHandle node_handle, string robot_name):node_handle_(node_handle),
robot_name_(robot_name) {
		ROS_INFO("ROBOT_NAVIGATION Connecting to the simple database service");
	simple_database_client_=node_handle_.serviceClient<situation_assessment_msgs::QueryDatabase>("situation_assessment/query_database",true);
	simple_database_client_.waitForExistence();
	ROS_INFO("ROBOT_NAVIGATION Connected");
}

geometry_msgs::Pose DatabaseQueries::getPose(string name) {
	situation_assessment_msgs::QueryDatabase srv;

	situation_assessment_msgs::Fact fact;

	fact.model=robot_name_;
	fact.subject=name;
	fact.predicate.push_back("pose");
	srv.request.query=fact;

	geometry_msgs::Pose pose;
	if (simple_database_client_.call(srv)) {
		if (srv.response.result.size()==0) {
			ROS_ERROR("ROBOT_NAVIGATION Couldn't fine pose for %s",name.c_str());
		}
		else {
			if (srv.response.result[0].value.size()<6) {
				ROS_ERROR("ROBOT_NAVIGATION returned value but missing pose"); 
			}
			else {
			pose.position.x=boost::lexical_cast<double>(srv.response.result[0].value[0]);
			pose.position.y=boost::lexical_cast<double>(srv.response.result[0].value[1]);
			pose.position.z=boost::lexical_cast<double>(srv.response.result[0].value[2]);
			pose.orientation.x=boost::lexical_cast<double>(srv.response.result[0].value[3]);
			pose.orientation.y=boost::lexical_cast<double>(srv.response.result[0].value[4]);
			pose.orientation.z=boost::lexical_cast<double>(srv.response.result[0].value[5]);
			pose.orientation.w=boost::lexical_cast<double>(srv.response.result[0].value[6]);
			}
		}
	}
	else {
		ROS_ERROR("Could not contact database");
	}
	return pose;
}

bool DatabaseQueries::hasArea(string name) {
	situation_assessment_msgs::QueryDatabase srv;
	situation_assessment_msgs::Fact fact;

	fact.model=robot_name_;
	fact.subject=name;
	fact.predicate.push_back("hasArea");
	srv.request.query=fact;;


	if (simple_database_client_.call(srv)) {
		if (srv.response.result.size()>0) {
			return true;
		}
	}
	else {
		ROS_ERROR("Could not contact database");
	}
	return false;
}

string DatabaseQueries::getEntityType(string name) {
	situation_assessment_msgs::QueryDatabase srv;
	situation_assessment_msgs::Fact fact;

	fact.model=robot_name_;
	fact.subject=name;
	fact.predicate.push_back("type");
	srv.request.query=fact;

	if (simple_database_client_.call(srv)) {
		if (srv.response.result.size()>0) {
			if (srv.response.result[0].value.size()>0) {
				return srv.response.result[0].value[0];
			}
			else {
				ROS_ERROR("ROBOT_NAVIGATION received response from db with no value");
			}
		}
		else {
			ROS_ERROR("ROBOT_NAVIGATION received response from db but no results");
		}
	}
	else {
		ROS_ERROR("Could not contact database");
	}
	return "";	
}

string DatabaseQueries::getRobotLocation(vector<string> robot_areas) {
	// ROS_INFO("ROBOT_NAVIGATION Robot areas size %ld",robot_areas_.size());
	for (int i=0; i<robot_areas.size();i++) {
		if (robot_areas[i]!="this") {

			situation_assessment_msgs::QueryDatabase srv;

			situation_assessment_msgs::Fact fact;

			fact.model=robot_name_;
			fact.subject=robot_areas[i];
			fact.predicate.push_back("type");
			srv.request.query=fact;


			if (simple_database_client_.call(srv)) {
				if (srv.response.result.size()>0) {
					if (srv.response.result[0].value.size()>0) {
						// ROS_INFO("ROBOT_NAVIGATION %s area is type %s",robot_areas_[i].c_str(),srv.response.result[0].value[0].c_str());
						if (srv.response.result[0].value[0]=="location") {
							ROS_INFO("ROBOT_NAVIGATION found robot location");
							return robot_areas[i];
						}
					}
					else {
						ROS_ERROR("ROBOT_NAVIGATION received result but no values");
					}
				}
				else {
					ROS_ERROR("Could not contact database");
				}
			}
		}	
	}
	return "this";			
}

geometry_msgs::Pose DatabaseQueries::getRobotPose() {
	situation_assessment_msgs::QueryDatabase srv;
	geometry_msgs::Pose robot_pose;

	situation_assessment_msgs::Fact fact;

	fact.model=robot_name_;
	fact.subject=robot_name_;
	fact.predicate.push_back("pose");
	srv.request.query=fact;
	if (simple_database_client_.call(srv)) {
		if (srv.response.result.size()>0) {
			if (srv.response.result[0].value.size()>3) {
				robot_pose.position.x=boost::lexical_cast<double>(srv.response.result[0].value[0]);
				robot_pose.position.y=boost::lexical_cast<double>(srv.response.result[0].value[1]);
				robot_pose.position.z=boost::lexical_cast<double>(srv.response.result[0].value[2]);

			}
			else {
				ROS_ERROR("ROBOT_NAVIGATION not enough values in pose");
			}
		}
		else  {
			ROS_ERROR("ROBOT_NAVIGATION no robot pose found");
		}
	}
	else {
		ROS_ERROR("ROBOT_NAVIGATION could not contact database");
	}
	return robot_pose;
}

string DatabaseQueries::getEntityLocation(string name) {
	vector<string> entity_areas;
	situation_assessment_msgs::QueryDatabase srv;

	situation_assessment_msgs::Fact fact;

	fact.model=robot_name_;
	fact.subject=name;
	fact.predicate.push_back("isInArea");
	srv.request.query=fact;

	if (simple_database_client_.call(srv)) {

		if (srv.response.result.size()>0) {


			entity_areas=srv.response.result[0].value;

			for (int i=0; i<entity_areas.size();i++) {
				if (entity_areas[i]!="this") {
		 			fact.predicate.clear();

					fact.model=robot_name_;
					fact.subject=entity_areas[i];
					fact.predicate.push_back("type");
					srv.request.query=fact;

					if (simple_database_client_.call(srv)) {
						if (srv.response.result.size()>0) {
							if (srv.response.result[0].value.size()>0) {
								if (srv.response.result[0].value[0]=="location") return entity_areas[i];
							}
							else {
								ROS_ERROR("ROBOT_NAVIGATION no values returned from db");
							}
						}
					}
					else {
						ROS_ERROR("Could not contact database");
						break;
					}
				}
			} 
		}
		else {
			ROS_ERROR("ROBOT_NAVIGATION received response without results from db");
		}
	}
	else {
		ROS_ERROR("Could not contact database");
	}
	return "this";
}