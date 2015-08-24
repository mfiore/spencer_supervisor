/**
  map_info.h
   Created on: Apr 19, 2015
       Author: mfiore
 
 Contains information about a map. The spencer project uses several small maps, representing different nodes in the 
 environment.
 	
 */

#ifndef SOURCE_DIRECTORY__SPENCER_SPENCER_MAP_SERVER_SRC_MAPINFO_H_
#define SOURCE_DIRECTORY__SPENCER_SPENCER_MAP_SERVER_SRC_MAPINFO_H_

#include <string>
#include <vector>
using namespace std;

class MapInfo {
public:
	MapInfo();
	MapInfo(const MapInfo& other);
	virtual ~MapInfo();

	string name;   //name of the node
	//coordinates of the origin
	double originX;  
	double originY;

	//vertexs of the node
	vector<double> vertexX;
	vector<double> vertexY;

	//centroid of the node
	double centerX;
	double centerY;


};

#endif /* SOURCE_DIRECTORY__SPENCER_SPENCER_MAP_SERVER_SRC_MAPINFO_H_ */
