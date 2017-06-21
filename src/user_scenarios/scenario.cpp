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
//	std::vector<Agent*> agentlist1;
//	std::vector<Agent*> agentlist2;


	//Initialize: Ardrone instance: ardrone1
	Ardrone_20170227* ardrone0=new Ardrone_20170227(agentlist.size());

	/* Agent parameters:
	 * {  lateralA,lateralB,verticalA,verticalB,orientationalA,orientationalB,
	 *    qposlat,qposlat,qposvert,qposorient,qposorient,qvlat,qvlat,qvvert,qvorient,
	 *    ruvlat,ruvlat,ruvver,ruorient
	 *  } dim[19]
	*/
	/*
	double ardrone0_init_p[]={
			-0.5092,1.458,3,3,1,1.3,
			1.0,1.0,1.0, 1.0,1.0, 0.0,0.0,0.0,0.0,
			1.0,1.0,1.0,1.0
	};
	*/
	double ardrone0_init_p[]={ //without tracking position
			-0.5092,1.458,-1,1,-5,1.3,
			0.0,0.0,0.0, 0.0,0.0, 0.0,0.0,0.0,0.0,
			1.0,1.0,0.3,1.0
	};
	double ardrone0_init_x[]=   {0,0,2, 1,0, 0.0,0.0,0.0,0.0};
	double ardrone0_init_xdes[]={0,0,2, 1,0, 0.0,0.0,0.0,0.0};
	ardrone0->setInitialState(ardrone0_init_x);
	ardrone0->setInitialDesiredState(ardrone0_init_xdes);
	ardrone0->setInitialParameter(ardrone0_init_p);
	ardrone0->setStateSubscriberRosTopicName       ("/Ardrone2SimpleLinModel_HASHMARK_0/pose");
	ardrone0->setDesiredStateSubscriberRosTopicName("/Ardrone2SimpleLinModel_HASHMARK_0/desiredpose");
	ardrone0->setPublisherRosTopicName             ("/Ardrone2SimpleLinModel_HASHMARK_0/ext_ctrl_ch");
	agentlist.push_back(ardrone0); /*add to agentlist*/

	//AddConstraint
	std::vector<Controller*> constraintlist;
	Constraint* constraint= new OrientationConstraint_20170227(ardrone0);

	//constraint_init_p{k0, ds, beta, ddist, kforw, kside, kup}
	double constraint0_init_p[]={1,0.17,0.5,1.5,1,1,1};
	constraint->setInitialParameter(constraint0_init_p);
	constraint->setParameter(constraint0_init_p);



/****** Initialize Coupling Instances ******/
	std::vector<Controller*> controllerlist;

	//Initialize: Controller
	Cmscgmres* controller1=new Cmscgmres(agentlist,controllerlist.size());
	controller1->setHorizonDiskretization(10);
	controller1->setHorizonLength(1);
	controller1->setTolerance(1e-8);
	controller1->setUpdateIntervall(0.01);
	controller1->setMaximumNumberofIterations(10);
	controller1->activateInfo_ControllerStates();
	//	controller1->activateInfo_ControllerTrace();
	//	controller1->activateInfo_Controller();
	//	controller1->startLogging2File();
	controllerlist.push_back(controller1);


/****** Initialize Events ******/
	std::vector<Event*> eventlist;

/****** Initialize Scheduler ******/
	Scheduler scheduler(argc,argv,controllerlist, eventlist);
//	scheduler.run_control(0.01);
	scheduler.run_vrep(0.01);
//	scheduler.run_simulation(0.1,10);

};



