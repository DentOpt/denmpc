/**
 * @file    Scheduler.h
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Scheduler Container
 *
 * Scheduler is managing the communication and control update intervalls.
 * It also allows switching between control, simulation and V-Rep mode.
 */

#ifndef _SCHEDULER_H
#define _SCHEDULER_H

#include <ros/ros.h>	//For ros::

#include "dentopt_nmpc_controller/VrepInfo.h"
#include <boost/filesystem.hpp>
#include "Event.h"
#include "Controller.h"
#include "Agent.h"
#include "Constraint.h"
#include "Controller.h"
#include <vector>		//For vector
#include <stdio.h> 		//For printf



class Scheduler {
protected:
	/*Control update intervall*/
	double time_intervall_;
	/*Global time in framework*/
	double time_framework_;
	/*Framework loop counter*/
	int counter_fccfloop_;
	/*std::vector of all agents*/
	std::vector<Agent*> agentlist_;
	std::vector<Event*> eventlist_;
	std::vector<Controller*> controllerlist_;
	/*Framework calibration time*/
	double ros_time_calibration_;

	Controller* ptr_controller;

	/*ROS control loop scheduler*/
	void controlloop();

	/*Simulation control loop scheduler*/
	void simulationloop(double simtime);

	/*VREP control loop scheduler (via ros)*/
	void vrepcontrolloop();
	void vrepsimrunning(const dentopt_nmpc_controller::VrepInfo& msgInfo);
	float vrep_time_;
	bool  vrep_flag_stop_;
	bool  vrep_flag_pause_;
	bool  vrep_flag_realtime_;

public:
	//	Scheduler(int argc, char **argv, std::vector<Agent*> _agentlist, std::vector<Constraint*> constraintlist);
	Scheduler(
			int argc,
			char **argv,
			std::vector<Controller*> controllerlist,
			std::vector<Event*> eventlist
			);
	void run_control(double dt){
		time_intervall_=dt;
		controlloop();
	}
	void run_vrep(double dt){
		time_intervall_=dt;
		vrepcontrolloop();
	}
	void run_simulation(double dt, double simtime){
		time_intervall_=dt;
		simulationloop(simtime);
	}


};

#endif //_Scheduler_H
