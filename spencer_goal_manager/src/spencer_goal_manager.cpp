#include <ros/ros.h>

#include <string>
#include <vector>
#include <supervision_msgs/GuideGroupAction.h>
#include <supervision_msgs/MoveToAction.h>
#include <supervision_msgs/SupervisionStatus.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 


#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>


using namespace std;

typedef actionlib::SimpleActionClient<supervision_msgs::GuideGroupAction> GuideGroupClient; 
typedef actionlib::SimpleActionClient<supervision_msgs::MoveToAction> MoveToClient; 

//actions
GuideGroupClient* guide_group_client_;
MoveToClient* move_to_client_;

//control variables and mutexs
bool is_guiding_=false;
boost::mutex mutex_guide_group_;
boost::condition_variable guide_group_condition_;

boost::mutex mutex_stopped_;
bool stopped_;

boost::mutex mutex_should_quit_;
bool should_quit_;


//sockets
int client_command_socket_,client_change_socket_;

int port_command=1438;
int port_change=1439;

//publishers
ros::Publisher status_pub;

//Control functions, thread safe
bool isStopped() {
	boost::lock_guard<boost::mutex> lock(mutex_stopped_);
	return stopped_;
}

void setStopped(bool value) {
	boost::lock_guard<boost::mutex> lock(mutex_stopped_);
	stopped_=value;
}

bool shouldQuit() {
	boost::lock_guard<boost::mutex> lock(mutex_should_quit_);
	return should_quit_;
}

void setShouldQuit(bool value) {
	boost::lock_guard<boost::mutex> lock(mutex_should_quit_);
	should_quit_=value;
}

//utility function to send a resuòt
void sendResult(string answer, int socket) {
	int	n = write(socket,answer.c_str(),answer.size());
	if (n < 0) {
	   	ROS_ERROR("SPENCER_GOAL_MANAGER ERROR writing to socket");
	}
}

//guides a group to a destination. Send_result can be useful if we want the function to be a "main called function" or an sub function
//used by other requests (which will send results back to the client)
void guideGroup(string gate, bool send_result) {
	supervision_msgs::GuideGroupGoal guide_goal;

	// supervision_msgs::SupervisionStatus status_msg; 
	// status_msg.status="Guiding group to "+gate;
	// status_pub.publish(status_msg);

	guide_goal.destination=gate;	

	guide_group_client_->sendGoal(guide_goal);
	mutex_guide_group_.lock();
	is_guiding_=true;
	mutex_guide_group_.unlock();

	bool has_finished=false;
	while (!has_finished && !isStopped()) {
		has_finished=guide_group_client_->waitForResult(ros::Duration(0.3));
	}
	if (!has_finished) setStopped(false);
	boost::unique_lock<boost::mutex> lock(mutex_guide_group_);
	is_guiding_=false;
	guide_group_condition_.notify_one();

	if (send_result) {
		if (!has_finished) {
			sendResult("STOPPED",client_command_socket_);
		}
		else {
			sendResult(guide_group_client_->getResult()->status,client_command_socket_);
		}
	}
}

bool moveToDestination(string dest,bool send_result) {
	supervision_msgs::MoveToGoal move_to_goal;

	// supervision_msgs::SupervisionStatus status_msgs
	// status_msg.status="Moving to "+dest;
	// status_pub.publish(status_msg);

	move_to_client_->sendGoal(move_to_goal);
	bool has_finished=false;
	while (!has_finished && !isStopped()) {
		has_finished=move_to_client_->waitForResult(ros::Duration(0.3));
	}
	if (!has_finished) setStopped(false);
	if (send_result) {
		if (!has_finished) {
			sendResult("STOPPED",client_command_socket_);
		}
		else {
			sendResult(move_to_client_->getResult()->status,client_command_socket_);
		}
	}
	if (has_finished) {
		return move_to_client_->getResult()->status=="OK";
	}
	else {
		return false;
	}
}

bool informAndGuide(string old_gate, string new_gate) {
	supervision_msgs::MoveToGoal move_to_goal;
	move_to_goal.destination="old_gate";

	bool has_moved=moveToDestination(old_gate,false);

	if (has_moved) {
		//inform routine
		ros::Duration(60).sleep();
		guideGroup(new_gate,true);
	}
	else {
		sendResult("FAILURE",client_command_socket_);
	}
}


void changeGate(string new_gate) {
	supervision_msgs::GuideGroupGoal guide_goal;

	guide_goal.destination=new_gate;

	supervision_msgs::SupervisionStatus status_msg; 
	status_msg.status="Changing gate to "+new_gate;
	status_pub.publish(status_msg);

	if (guide_group_client_->getState()==actionlib::SimpleClientGoalState::ACTIVE) {
		guide_group_client_->cancelGoal();
	}

	boost::unique_lock<boost::mutex> lock(mutex_guide_group_);
	while (is_guiding_) {
		guide_group_condition_.wait(lock);
	}
	lock.unlock();

	guideGroup(new_gate,true);
}


vector<string> getParameters(string command) {
	vector<string> parameters;

	int n1=command.find(" ",0);
	if (n1!=string::npos) {
		n1++;
		int n2;
		while (n2=command.find(" ",n1)!=string::npos) {
			string a_parameter=command.substr(n1,n2-n1);
			n1++;
			parameters.push_back(a_parameter);
		}
		string a_parameter=command.substr(n1);
		parameters.push_back(a_parameter);
	}
	return parameters;
}


void commandLoop() {
	 	int server_socket;
	    socklen_t clilen;
	    char buffer[256];
	    struct sockaddr_in serv_addr, cli_addr;
	    int n;
	    server_socket = socket(AF_INET, SOCK_STREAM, 0);
	    if (server_socket < 0) {
	        ROS_ERROR("SPENCER_GOAL_MANAGER ERROR opening socket");
	        ros::shutdown();
	        return;
	    }
	    bzero((char *) &serv_addr, sizeof(serv_addr));
	    serv_addr.sin_family = AF_INET;
	    serv_addr.sin_addr.s_addr = INADDR_ANY;
	    serv_addr.sin_port = htons(port_command);
	    if (bind(server_socket, (struct sockaddr *) &serv_addr, 
	              sizeof(serv_addr)) < 0)  {
	       ROS_ERROR("SPENCER_GOAL_MANAGER ERROR on binding");
	       ros::shutdown();
	       return;
	     }
	     listen(server_socket,5);
	     clilen = sizeof(cli_addr);
	     client_command_socket_ = accept(server_socket, 
	                 (struct sockaddr *) &cli_addr, 
	                 &clilen);
	     if (client_command_socket_ < 0)  {
	          ROS_ERROR("SPENCER_GOAL_MANAGER ERROR on accept");
	          ros::shutdown();
	          return;
	     }
	     bzero(buffer,256);

	     string command;
	     string error;


	     while (command!="exit" && ros::ok()) {
		     n = read(client_command_socket_,buffer,255);
		     if (n < 0) {
		     	ROS_ERROR("SPENCER_GOAL_MANAGER ERROR reading socket");
		     	ros::shutdown();
		     	return; 
		     }
		     ROS_INFO("SPENCER_GOAL_MANAGER Here is the message: %s\n",buffer);
		     string s(buffer);

		     vector<string> parameters;
		     if (s=="move") {
		     	parameters=getParameters(s);
		     	if (parameters.size()==1) {
		     		boost::thread t(&moveToDestination,parameters[0],true);
		     	}
		     	else {
					ROS_ERROR("SPENCER_GOAL_MANAGER WRONG_PARAMETERS");
		     		sendResult("WRONG_PARAMETERS",client_command_socket_);		     	}
		     }
		     else if (s=="mode") {

		     }
		     else if (s=="inform_gate_change") {
		     	parameters=getParameters(s);
		     	if (parameters.size()==2) {
		     		boost::thread t(&informAndGuide,parameters[0],parameters[1]);
		     	}	     	
		     	else {
		     		ROS_ERROR("SPENCER_GOAL_MANAGER WRONG_PARAMETERS");
		     		sendResult("WRONG_PARAMETERS",client_command_socket_);
		     	}
		     }
	 	}
	 	close(server_socket);
	 	close(client_command_socket_);
}

void changeLoop() {
   int server_socket;
   socklen_t clilen;
   char buffer[256];
   struct sockaddr_in serv_addr, cli_addr;
   int n;
   server_socket = socket(AF_INET, SOCK_STREAM, 0);
   if (server_socket < 0) {
       ROS_ERROR("SPENCER_GOAL_MANAGER ERROR opening socket");
       ros::shutdown();
       return;
   }
   bzero((char *) &serv_addr, sizeof(serv_addr));
   serv_addr.sin_family = AF_INET;
   serv_addr.sin_addr.s_addr = INADDR_ANY;
   serv_addr.sin_port = htons(port_change);
   if (bind(server_socket, (struct sockaddr *) &serv_addr, 
             sizeof(serv_addr)) < 0)  {
      ROS_ERROR("SPENCER_GOAL_MANAGER ERROR on binding");
      ros::shutdown();
      return;
    }
    listen(server_socket,5);
    clilen = sizeof(cli_addr);
    client_change_socket_ = accept(server_socket, 
                (struct sockaddr *) &cli_addr, 
                &clilen);
    if (client_change_socket_ < 0)  {
         ROS_ERROR("SPENCER_GOAL_MANAGER ERROR on accept");
         ros::shutdown();
         return;
    }
    bzero(buffer,256);


    string command;
    string error;

    while (!shouldQuit()) {
	    n = read(client_change_socket_,buffer,255);
		if (n < 0) {
		   	ROS_ERROR("SPENCER_GOAL_MANAGER ERROR reading socket");
		   	ros::shutdown();
		   	return; 
		}
		ROS_INFO("SPENCER_GOAL_MANAGER change message is: %s\n",buffer);
		string s(buffer);
		if (s=="STOP") {
			setStopped(true);
		}
		else if (s=="GATE_CHANGE") {
			vector<string> parameters=getParameters(s);
			if (parameters.size()==1) {
				boost::thread t(&changeGate,parameters[0]);
			}
			else {
				ROS_ERROR("SPENCER_GOAL_MANAGER WRONG_PARAMETERS");
				sendResult("WRONG_PARAMETERS",client_change_socket_);
			}
		}
		sendResult("OK",client_change_socket_);
	}
	close(server_socket);
	close(client_change_socket_);
}


int main(int argc, char** argv) {
	ros::init(argc,argv,"spencer_goal_manager");
	ros::NodeHandle node_handle;

	guide_group_client_=new GuideGroupClient("supervision/guide_group",true);
	move_to_client_=new MoveToClient("supervision/move_to",true);

	guide_group_client_->waitForServer();
	move_to_client_->waitForServer();

	// status_pub=node_handle.advertise<supervision_msgs::SupervisionStatus>("supervision/goal",1000);

	boost::thread command_thread(&commandLoop);
	boost::thread change_thread(&changeLoop);

	command_thread.join();
	setShouldQuit(true);
	change_thread.join();

 	close(client_command_socket_);
 	ros::shutdown();
    return 0; 
}
