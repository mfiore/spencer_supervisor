/**
  spencer_map.h
  Author: Michelangelo Fiore
 
  Takes care of the set of spencer maps, by collecting their data from yaml files and providing access
  to them for the supervisor.
 */

#ifndef SPENCER_MAP_H
#define SPENCER_MAP_H

//ros
#include <ros/ros.h>
#include <annotated_mapping/SwitchMap.h>
#include <situation_assessment_msgs/AddArea.h>

//other
#include <string>
#include <vector>
#include <map>
#include <utility>

//self
#include "robot_navigation/map_info.h"
#include "tinyxml2.h"

 //boost
#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <boost/lexical_cast.hpp>


using namespace std;

class SpencerMap {
public:
		SpencerMap(ros::NodeHandle node_handle,string doc_path,string doc_name);
		//returns in dx,dy the coordinates of the center of the map. Returns false if node is not found
		bool getMapCenter(string node, double *dx, double *dy);  
		bool calculateMapInfos(); //must be called at the start. Calculates information of the map from the yaml files

private:
		//paths to the main and sub yaml files for the map
		string doc_path_, doc_name_;
		map<string,MapInfo> mapInfo; //keeps information of the map

		ros::NodeHandle node_handle_;
		ros::ServiceClient addAreaClient;

};

#endif