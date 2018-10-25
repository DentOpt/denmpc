/**
 * @file    scenario.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   User Scenario
 *
 * Scenario is defining containing int main() and defining the control scenario
 */

#include "Scheduler.h"
#include "Agent.h"
#include "Constraint.h"
#include "Coupling.h"

#include "Ardrone_20170227.h"
#include "OrientationConstraint_20170227.h"
#include "CollisionAvoidanceCoupling_20170227.h"

#include "Cmscgmres.h"
#include "Event.h"
#include "std_msgs/Bool.h"

int main(int argc, char **argv){

//Initialize ros node
ros::init(argc, argv, "controller");

/****** Initialize Agent Instances ******/
	std::vector<Agent*> agentlist;

	//Initialize: Ardrone instance: ardrone1
	Ardrone_20170227* ardrone0=new Ardrone_20170227(agentlist.size());
	agentlist.push_back(ardrone0); /*add to agentlist*/

/****** Initialize Coupling Instances ******/
	std::vector<Controller*> controllerlist;

	//Initialize: Controller
	Cmscgmres* controller1=new Cmscgmres(agentlist,controllerlist.size());
	controller1->activateInfo_ControllerStates();
	controllerlist.push_back(controller1);

/****** Initialize Events ******/
	std::vector<Event*> eventlist;

/****** Initialize Scheduler ******/
	Scheduler scheduler(argc,argv,controllerlist, eventlist);
	scheduler.run_control(0.01);
};



