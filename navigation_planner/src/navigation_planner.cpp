/**
	navigation_planner.cpp
	author: Michelangelo Fiore

	This class plans a path in the symbolic map. The implementation uses the boost graphs and dijkstra algorithm.
*/

#include <ros/ros.h>

#include <situation_assessment_msgs/Graph.h>
#include <situation_assessment_msgs/Node.h>
#include <situation_assessment_msgs/Edge.h>
#include <situation_assessment_msgs/Property.h>
#include <situation_assessment_msgs/GetMap.h>

#include <supervision_msgs/CalculatePath.h>
#include <supervision_msgs/GetConnectedNodes.h>

#include <string>
#include <vector>
#include <map>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/foreach.hpp>

using namespace std;

//useful typedef since boost graphs are kinda complex
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, 
                       boost::no_property, boost::property<boost::edge_weight_t, int> > Graph;

typedef std::pair<int, int> Edge;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;


//predecessor map for a graph.
template <class PredecessorMap>
  class record_predecessors : public boost::dijkstra_visitor<>
  {
  public:
    record_predecessors(PredecessorMap p)
      : m_predecessor(p) { }

    template <class Edge, class Graph>
    void edge_relaxed(Edge e, Graph& g) {
      // set the parent of the target(e) to source(e)
      put(m_predecessor, boost::target(e, g), boost::source(e, g));
    }
  protected:
    PredecessorMap m_predecessor;
  };


  template <class PredecessorMap>
  record_predecessors<PredecessorMap>
  make_predecessor_recorder(PredecessorMap p) {
    return record_predecessors<PredecessorMap>(p);
  }


 Graph *symbolic_map;
 ros::ServiceClient map_client;

 map<string,int> label_to_index;
 vector<string> labels;

//gets the symbolic map, and creates a graph to represent it
bool getMap() {
	situation_assessment_msgs::GetMap get_map_request;
	if (map_client.call(get_map_request)) {
		ROS_INFO(" NAVIGATION_PLANNER Got map");
		situation_assessment_msgs::Graph map_graph=get_map_request.response.graph;
	
		int n_nodes=map_graph.nodes.size();
		ROS_INFO(" NAVIGATION_PLANNER There are %d nodes:",n_nodes);
		for (int i=0;i<n_nodes;i++) {
			situation_assessment_msgs::Node node=map_graph.nodes[i];
			label_to_index[node.label]=i;
			labels.push_back(node.label);
			ROS_INFO(" NAVIGATION_PLANNER - %s",node.label.c_str());
		}

		vector<Edge > map_edges;
		vector<boost::property<boost::edge_weight_t, int> > weights;
		ROS_INFO(" NAVIGATION_PLANNER With edges:");
		for (int i=0; i<map_graph.edges.size();i++) {
			situation_assessment_msgs::Edge edge=map_graph.edges[i];
			// boost::graph::add_edge(label_to_index[edge.source],label_to_index[edge.destination],g);
			pair<int,int> pair_edge(label_to_index[edge.source],label_to_index[edge.destination]);
			map_edges.push_back(pair_edge);
			weights.push_back(1);
			pair<int,int> pair_edge2(label_to_index[edge.destination],label_to_index[edge.source]);
			map_edges.push_back(pair_edge2);
			ROS_INFO(" NAVIGATION_PLANNER - %s %s",edge.source.c_str(),edge.destination.c_str());
			weights.push_back(1);
		}
		symbolic_map=new Graph(map_edges.begin(),map_edges.end(),weights.begin(),n_nodes);


		return true;
	}
	else {
		return false;
	}
}

bool getConnectedNodes(supervision_msgs::GetConnectedNodes::Request &req, supervision_msgs::GetConnectedNodes::Response &res) {
	int source_node=label_to_index[req.node];
	vector<string> connected_nodes;
// 

	IndexMap index;

	boost::graph_traits<Graph>::adjacency_iterator ai;
	boost::graph_traits<Graph>::adjacency_iterator ai_end;
	for (tie(ai, ai_end) = boost::adjacent_vertices(source_node, *symbolic_map); 
	           ai != ai_end; ++ai) {
	         connected_nodes.push_back(labels[index[*ai]]);
	}
	res.connected_nodes=connected_nodes;
	return true;
}

//calculates  a path in the map. Every node has a distance of one at the moment.
bool calculatePath(supervision_msgs::CalculatePath::Request &req, supervision_msgs::CalculatePath::Response &res) {
	int source_node=label_to_index[req.source];
	int dest_node=label_to_index[req.dest];

	// vector for storing distance and predecessor property
	 std::vector<int> d(num_vertices(*symbolic_map));
	 vector<Vertex> p(num_vertices(*symbolic_map), boost::graph_traits<Graph>::null_vertex()); //the predecessor array

	 // get the source node
	 Vertex s = *(vertices(*symbolic_map).first+source_node);

	 //invoke algoritm
	 dijkstra_shortest_paths(*symbolic_map, s, boost::distance_map(&d[0]).
	 	visitor(make_predecessor_recorder(&p[0])));

	 IndexMap index;

	 // ROS_INFO(" NAVIGATION_PLANNER distances from start vertex:");
	 boost::graph_traits<Graph>::vertex_iterator vi;
	 for(vi = vertices(*symbolic_map).first; vi != vertices(*symbolic_map).second; ++vi)
	   // ROS_INFO(" NAVIGATION_PLANNER distance(%ld) = %d",index(*vi),d[*vi]);

	 // ROS_INFO(" NAVIGATION_PLANNER parents in the tree of shortest paths:");
	   for(vi = boost::vertices(*symbolic_map).first; vi != boost::vertices(*symbolic_map).second; ++vi) {
	     if (p[*vi] == boost::graph_traits<Graph>::null_vertex())
	       // ROS_INFO(" NAVIGATION_PLANNER Parent(%ld) = no parent",index(*vi)); 
	     else 
	       //ROS_INFO(" NAVIGATION_PLANNER Parent(%ld) = %ld",index(*vi),p[*vi]);
	   }

   boost::graph_traits<Graph>::vertex_iterator v_start=boost::vertices(*symbolic_map).first+source_node;


   int distance;
   vector<string> predecessors;

   bool found_path=true;;
    ROS_INFO(" NAVIGATION_PLANNER Predecessors ");
	vi=boost::vertices(*symbolic_map).first+dest_node;
	distance=d[*vi];
	while (vi!=v_start) {
		if (p[*vi] == boost::graph_traits<Graph>::null_vertex()) {
			ROS_WARN("No path found");
			found_path=false;
			break;
		}
		else {
			ROS_INFO(" NAVIGATION_PLANNER - %s",labels[index(*vi)].c_str());
			predecessors.push_back(labels[index(*vi)]);
			vi=vertices(*symbolic_map).first+p[*vi];
		}
	}	 
	if (found_path) {
		ROS_INFO(" NAVIGATION_PLANNER - %s", labels[index(*vi)].c_str());
		predecessors.push_back(labels[index(*v_start)]);

		if (predecessors.size()>0) {
		std::reverse(predecessors.begin(),predecessors.end());
		}
	}
	res.path=predecessors;
	res.distance=distance;
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc,argv,"navigation_planner");
	ros::NodeHandle node_handle;

	ROS_INFO(" NAVIGATION_PLANNER Init navigation_planner");

	map_client=node_handle.serviceClient<situation_assessment_msgs::GetMap>("situation_assessment/get_symbolic_map");
	ROS_INFO(" NAVIGATION_PLANNER Waiting for get map service");
	map_client.waitForExistence();

	if (!getMap()) {
		ROS_ERROR("Failed to get map");
		ros::shutdown();
	}
	
	ros::ServiceServer path_server=node_handle.advertiseService("supervision/calculate_path",calculatePath);
	ros::ServiceServer connected_nodes_server=node_handle.advertiseService("supervision/get_connected_nodes",getConnectedNodes);

	ROS_INFO(" NAVIGATION_PLANNER Advertising calculate path");
	ROS_INFO(" NAVIGATION_PLANNER Ready");
	ros::spin();
}