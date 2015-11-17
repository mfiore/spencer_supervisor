/**
  @brief Controller console for APPL&ROS wrapper
  @author Shaoujun Cai (based on Haoyu Bai's work)
  @date 2013-05-08
 **/


#include "ros/ros.h"
#include "appl/GetAction.h"
#include "Controller.h"
#include "GlobalResource.h"
#include "MOMDP.h"
#include "ParserSelector.h"
#include "AlphaVectorPolicy.h"
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <sstream>
using namespace std;

Controller *p_control;
map<string, int> obsSymbolMapping, obsStateMapping;
SharedPointer<MOMDP> problem;





/*service callback function to process the command from client*/
bool servicefn(appl::GetAction::Request &req, appl::GetAction::Response &res) {
    ROS_INFO("Received request");
    //command 0,1,2   ping init request
    int command = req.cmd;
    ROS_INFO("command %d", req.cmd);
    ROS_INFO("xstate %s", req.xstate.c_str());
    ROS_INFO("obs %s", req.obs.c_str());
    string xstate = req.xstate;
    string obs = req.obs;
    int action;
    if (command == 0) {
        res.action = -1; //for testing
    } else if (command == 1) {
        // format: ".init xstate"
        int idx_xstate = obsStateMapping[xstate];
        p_control->reset(idx_xstate);
        // dummy obs for first action
        action = p_control->nextAction(0, 0);
        res.action = action;
        cout << "action:" << action << endl;
        map<string, string> actState = problem->getActionsSymbols(action);
        for (map<string, string>::iterator i = actState.begin(); i != actState.end(); i++) {
            ROS_INFO("%s %s", i->first.c_str(), i->second.c_str());
        }
//        ROS_INFO("states");
        vector<string> ystate;
        int mostProbY = p_control->currBelief()->bvec->argmax(); //get the most probable Y state
        map<string, string> mostProbYState = problem->getFactoredUnobservedStatesSymbols(mostProbY);
        for (map<string, string>::iterator i = mostProbYState.begin(); i != mostProbYState.end(); i++) {
//            ROS_INFO("%s %s", i->first.c_str(), i->second.c_str());
            ystate.push_back(i->second);
        }
        res.ystate = ystate;

    } else {
        int idx_xstate = obsStateMapping[xstate];
        int idx_obs = obsSymbolMapping[obs];
        action = p_control->nextAction(idx_obs, idx_xstate);
        res.action = action;
        cout << "action:" << action << endl;
        map<string, string> actState = problem->getActionsSymbols(action);
        for (map<string, string>::iterator i = actState.begin(); i != actState.end(); i++) {
            ROS_INFO("%s %s", i->first.c_str(), i->second.c_str());
        }
//        ROS_INFO("states");
        vector<string> ystate;
        int mostProbY = p_control->currBelief()->bvec->argmax(); //get the most probable Y state
        map<string, string> mostProbYState = problem->getFactoredUnobservedStatesSymbols(mostProbY);
        for (map<string, string>::iterator i = mostProbYState.begin(); i != mostProbYState.end(); i++) {
//            ROS_INFO("%s %s", i->first.c_str(), i->second.c_str());
            ystate.push_back(i->second);
        }
        res.ystate = ystate;
        //printTuple(actState, streamOut);
    }
    cout << "Current belief:\n";
    //    p_control->currBelief()->bvec->write(cout);
    for (std::vector<SparseVector_Entry>::const_iterator iter = p_control->currBelief()->bvec->data.begin();
            iter != p_control->currBelief()->bvec->data.end(); iter++) {
        map<string, string> yState = problem->getFactoredUnobservedStatesSymbols(iter->index);
        for (map<string, string>::iterator i = yState.begin(); i != yState.end(); i++) {
            ROS_INFO("%s %s", i->first.c_str(), i->second.c_str());
            
        }
        ROS_INFO("prob %f",iter->value);
    }
    cout << endl;

    ROS_INFO("action service ended");
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control_pipe");
    ros::NodeHandle nh;
    ros::NodeHandle nh2("~");


   
    std::string problem_name, policy_name, default_param;
    string name;
    nh2.getParam("problem_name", name);
    ROS_INFO("%s",name.c_str());
    nh.getParam("/supervision/"+name+"/appl_problem", problem_name);
    nh.getParam("/supervision/"+name+"/appl_policy", policy_name);
    ROS_INFO("%s!!!!!!!!!!!!!!!!!!", problem_name.c_str());
    ROS_INFO("%s",policy_name.c_str());


    SolverParams* p = &GlobalResource::getInstance()->solverParams;
    setvbuf(stdout, 0, _IONBF, 0);
    setvbuf(stderr, 0, _IONBF, 0);
    p->policyFile = policy_name;
    p->problemName = problem_name;

    ROS_INFO("before loading the model");

    cout << "\nLoading the model ...\n   ";
    problem = ParserSelector::loadProblem(p->problemName, *p);

    SharedPointer<AlphaVectorPolicy> policy = new AlphaVectorPolicy(problem);

    cout << "\nLoading the policy ... input file : " << p->policyFile << "\n";
    bool policyRead = policy->readFromFile(p->policyFile);

    if (p->useLookahead) {
        cout << "   action selection : one-step look ahead\n";
    }

    // TODO we assume the initial observed state is 0 for now
    p_control = new Controller(problem, policy, p, 0);

    /// Mapping observations for quick reference

    for (int i = 0; i < problem->XStates->size(); i++) {
        //mapFile << "State : " << i <<  endl;
        //cout << "State : " << i <<  endl;
        map<string, string> obsState = problem->getFactoredObservedStatesSymbols(i);
        string state_str;
        for (map<string, string>::iterator iter = obsState.begin(); iter != obsState.end(); iter++) {
            //cout << iter->first << " : " << iter->second << endl ;
            state_str.append(iter->second);
        }
        obsStateMapping[state_str] = i;
        ROS_INFO("\n list of observable variables\n");
        ROS_INFO("\n %s %d", state_str.c_str(), i);
    }

    for (int i = 0; i < problem->observations->size(); i++) {
        map<string, string> obsSym = problem->getObservationsSymbols(i);
        string obs_str;
        for (map<string, string>::iterator iter = obsSym.begin(); iter != obsSym.end(); iter++) {
            obs_str.append(iter->second);
        }
        obsSymbolMapping[obs_str] = i;

        ROS_INFO("\n list of observations\n");
        ROS_INFO("\n %s %d", obs_str.c_str(), i);
    }


    cout << "\nReady.\n";

      ros::ServiceServer service = nh.advertiseService("appl_request/"+name, servicefn);

    ros::spin();


    return 0;
}
