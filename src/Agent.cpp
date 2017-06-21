/**
 * @file    Agent.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Agent Container
 *
 * Agent represents the container class for all types of agents
 * which can be any entity/system that can be described with an Differential equation.
 * E.g. robots, drones, plants, etc.
 */

#include "Agent.h"

	Agent::Agent(){
		std::cout<<"Agent::Agent()"<<std::endl;
	};
	Agent::Agent(int _id){
		std::cout<<"Agent::Agent(id)"<<std::endl;id_=_id;
	};
//	Agent::Agent(int _id,
//			std::vector<double> init_x,
//			std::vector<double> init_xdes,
//			std::vector<double> init_u,
//			std::vector<double> init_udes,
//			std::vector<double> init_y,
//			std::vector<double> init_ydes,
//			std::vector<double> init_p){
//		std::cout<<"Agent::Agent(id,x,xdes,u,udes,y,ydes,p)"<<std::endl;
//		id_=_id;
//	};
	Agent::~Agent(){
		//Deallocation of Memory
		MathLib::free(x_);
		MathLib::free(u_);
		MathLib::free(y_);
		MathLib::free(p_);
		MathLib::free(xdes_);
		MathLib::free(udes_);
		MathLib::free(ydes_);
		ros_publishers_.clear();
		ros_state_subscribers_.clear();
		ros_desired_state_subscribers_.clear();
		ros_desired_control_subscribers_.clear();
		ros_parameter_subscribers_.clear();
		constraint_.clear();
	}

void Agent::addConstraint(Constraint* _constraint){
		constraint_.push_back(_constraint);
}
void Agent::removeConstraint(Constraint* _constraint){
	//check arrays for the constraint and remove them
	constraint_.  erase(std::remove(constraint_.begin(),  constraint_.end(),  _constraint),constraint_.end());
}

void Agent::addCouplingAsAgent1(Coupling* _couplingconstraint){
	_couplingconstraint->setAgent1(this);
	coupling_.push_back(_couplingconstraint);
}
void Agent::addCouplingAsAgent2(Coupling* _couplingconstraint){
	_couplingconstraint->setAgent2(this);
	coupling_.push_back(_couplingconstraint);
}
void Agent::removeCoupling(Coupling* _couplingconstraint){
	//check arrays for the constraint and remove them
	coupling_.erase(std::remove(coupling_.begin(),  coupling_.end(),  _couplingconstraint),coupling_.end());
}


void Agent::addController(Controller* _controller){
	controller_.push_back(_controller);
}
void Agent::removeController(Controller* _controller){
	//check array for the controller and remove it
	controller_.erase(std::remove(controller_.begin(), controller_.end(), _controller), controller_.end());
}

//Set initial states to states
void Agent::reset2initialstate(){
	setState			(x_init_);
	setControl			(u_init_);
	setOutput			(y_init_);
	setDisturbance		(d_init_);
	setDesiredState		(xdes_init_);
	setDesiredControl	(udes_init_);
	setDesiredOutput	(ydes_init_);
	setParameter		(p_init_);
}
