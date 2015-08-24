/* 
spencer_map.cpp
author Michelangelo Fiore

*/

#include "robot_guide/spencer_map.h"

SpencerMap::SpencerMap(ros::NodeHandle node_handle, string doc_path, string doc_name):node_handle_(node_handle),doc_path_(doc_path),doc_name_(doc_name) {
	 ROS_INFO("Connecting to Add Area");
	 addAreaClient=node_handle_.serviceClient<situation_assessment_msgs::AddArea>("/situation_assessment/add_area",true);
	 addAreaClient.waitForExistence();
	 ROS_INFO("Connected");
}

//gets map informations from yaml files
bool SpencerMap::calculateMapInfos() {
	ROS_INFO("Calculating map information");

	tinyxml2::XMLDocument mainDoc,partsDoc;
	string main_doc_path,parts_doc_path;
	main_doc_path=doc_path_+doc_name_+".xml";
	parts_doc_path=doc_path_+doc_name_+"_parts.xml";
	int error1=mainDoc.LoadFile( main_doc_path.c_str() );
	int error2=partsDoc.LoadFile(parts_doc_path.c_str());

	if (error1!=0) {
		ROS_ERROR("Cannot find main map file");
		return false;
	}
	if (error2!=0) {
		ROS_INFO("Cannot find parts map file");
		return false;
	}
	else {
		ROS_INFO("Found maps. Starting parsing");
	}
	string resolutionString=mainDoc.FirstChildElement("project")->FirstChildElement("map")->FirstChildElement("resolution")->GetText();
	boost::trim(resolutionString);
	double resolution=boost::lexical_cast<double>(resolutionString);
	cout<<"resolution is "<<resolution<<"\n";

	string xString=mainDoc.FirstChildElement("project")->FirstChildElement("map")->FirstChildElement("origin")->FirstChildElement("x")->GetText();
	string yString=mainDoc.FirstChildElement("project")->FirstChildElement("map")->FirstChildElement("origin")->FirstChildElement("x")->GetText();

	boost::trim(xString);
	boost::trim(yString);

	double map_origin_x=boost::lexical_cast<double>(xString);
	double map_origin_y=boost::lexical_cast<double>(yString);

	tinyxml2::XMLNode *partNode=partsDoc.FirstChildElement("parts")->FirstChild();
	while (partNode!=NULL) {
		MapInfo aMap;

		string name=partNode->FirstChildElement("name")->GetText();

		string cx=partNode->FirstChildElement("center")->FirstChildElement("x")->GetText();
		string cy=partNode->FirstChildElement("center")->FirstChildElement("y")->GetText();
		boost::trim(cx);
		boost::trim(cy);

		double centerX=boost::lexical_cast<double>(cx);
		double centerY=boost::lexical_cast<double>(cy);

		aMap.name=name;
		aMap.centerX=80-(centerX*resolution+map_origin_x);
		aMap.centerY=80-(centerY*resolution+map_origin_y);

		tinyxml2::XMLNode *vertexNode=partNode->FirstChildElement("annotation")->FirstChildElement("vertex");
		vector<double> vertexX,vertexY;
		while (vertexNode!=NULL) {
			string svx=vertexNode->FirstChildElement("x")->GetText();
			string svy=vertexNode->FirstChildElement("y")->GetText();
			boost::trim(svx);
		    boost::trim(svy);
			double vx=boost::lexical_cast<double>(svx);
			double vy=boost::lexical_cast<double>(svy);
			vertexX.push_back(vx);
			vertexY.push_back(vy);
			vertexNode=vertexNode->NextSibling();
		}
		aMap.vertexX=vertexX;
		aMap.vertexY=vertexY;

		tinyxml2::XMLDocument partInfoNode;
		string partInfoName=doc_path_+doc_name_+"_"+name+".xml";
		ROS_INFO("Loading map %s",partInfoName.c_str());
		if (partInfoNode.LoadFile(partInfoName.c_str())!=0) {
			ROS_ERROR("Part document not found");
			return false;
		}
		tinyxml2::XMLNode *originNode=partInfoNode.FirstChildElement("map")->FirstChildElement("origin");

		string ox=originNode->FirstChildElement("x")->GetText();
		string oy=originNode->FirstChildElement("y")->GetText();
		boost::trim(ox);
		boost::trim(oy);
		double originX=boost::lexical_cast<double>(ox);
		double originY=boost::lexical_cast<double>(oy);
		aMap.originX=80-originX;
		aMap.originY=80-originY;

		mapInfo[name]=aMap;

		vector<geometry_msgs::Point32> vertexs;
		for (int i=0; i<vertexX.size();i++) {
			geometry_msgs::Point32 v;
			v.x=80-(vertexX[i]*resolution+map_origin_x);
			v.y=80-(vertexY[i]*resolution+map_origin_y);
			v.z=0;

			vertexs.push_back(v);
		}
		vertexs.push_back(vertexs[0]);

		geometry_msgs::Polygon polygon;
		polygon.points=vertexs;

		//monitors the area in situation assessment 

		situation_assessment_msgs::AddArea addAreaRq;
		addAreaRq.request.name=name;
		addAreaRq.request.area=polygon;

		addAreaClient.call(addAreaRq);
		ROS_INFO("Called addArea");

		partNode=partNode->NextSibling();
	}
	ROS_INFO("Parsed maps");
	return true;
}


bool SpencerMap::getMapCenter(string node, double *dx, double *dy) {
	*dx=mapInfo[node].centerX;
	*dy=mapInfo[node].centerY;

	return true;
}
