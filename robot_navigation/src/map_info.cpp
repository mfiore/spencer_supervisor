/*
 * map_info.cpp
 *
 *  Created on: Apr 19, 2015
 *      Author: mfiore
 */

#include "robot_navigation/map_info.h"

MapInfo::MapInfo() {
	// TODO Auto-generated constructor stub

}
MapInfo::MapInfo(const MapInfo& other) {
	this->centerX=other.centerX;
	this->centerY=other.centerY;
	this->name=other.name;
	this->originX=other.originX;
	this->originY=other.originY;
	this->vertexX=other.vertexX;
	this->vertexY=other.vertexY;
}

MapInfo::~MapInfo() {
	// TODO Auto-generated destructor stub
}

