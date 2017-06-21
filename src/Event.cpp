/**
 * @file    Event.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Event Container
 *
 * This can be used to implement events which handle the on-line
 * modularization of the control scenario, triggered by messages.
 * It covers adding and removing agents, constraints and couplings
 * from the scenario
 */

#include "Event.h"

/****************************************************
 * ROS EVENT
 ****************************************************/

/********************
 * Agent Events
 */
//	RosEvent_addAgent<T>::RosEvent_addAgent(Controller* controller, Agent* agent, double time):RosEvent<T>(time){
//		agent_	 =agent;
//		controller_=controller;
//	}
//	void RosEvent_addAgent<T>::eventAction(){
//		std::cout<<"RosEvent_addAgent::eventAction() "<<std::endl;
//		controller_->addAgent(agent_);
//	};
//
//	RosEvent_removeAgent<T>::RosEvent_removeAgent(Controller* controller, Agent* agent, double time):RosEvent<T>(time){
//		agent_	 =agent;
//		controller_=controller;
//	}
//	void RosEvent_removeAgent<T>::eventAction(){
//		std::cout<<"RosEvent_removeAgent::eventAction() "<<std::endl;
//		controller_->removeAgent(agent_);
//	};
//











	/****************************************************
	 * Time EVENT
	 ****************************************************/

/********************
 * Parameter Events
 */
	TimeEvent_changeParameters::TimeEvent_changeParameters(Agent* agent, double* p,double time):TimeEvent(time){
		agent_	   =agent;
		constraint_=NULL;
		coupling_  =NULL;
		parameters_=p;
	}
	TimeEvent_changeParameters::TimeEvent_changeParameters(Constraint* constraint, double* pc,double time):TimeEvent(time){
		agent_	    =NULL;
		constraint_ =constraint;
		coupling_   =NULL;
		parameters_ =pc;
	}
	TimeEvent_changeParameters::TimeEvent_changeParameters(Coupling* coupling, double* pc,double time):TimeEvent(time){
		agent_	   =NULL;
		constraint_=NULL;
		coupling_  =coupling;
		parameters_=pc;
	}
	void TimeEvent_changeParameters::eventAction(){
		if(agent_!=NULL){
			std::cout<<"Event::TimeEvent_addConstraint "<<std::endl;
			agent_->setParameter(parameters_);
			agent_->setInitialParameter(parameters_);
		}else if(constraint_!=NULL){
			std::cout<<"Event::TimeEvent_addConstraint "<<std::endl;
			constraint_->setParameter(parameters_);
			constraint_->setInitialParameter(parameters_);
		}if(coupling_!=NULL){
			std::cout<<"Event::TimeEvent_addConstraint "<<std::endl;
			coupling_->setParameter(parameters_);
			coupling_->setInitialParameter(parameters_);
		}
	};

/********************
 * Agent Events
 */
	TimeEvent_addAgent::TimeEvent_addAgent(Controller* controller, Agent* agent, double time):TimeEvent(time){
		agent_	 =agent;
		controller_=controller;
	}
	void TimeEvent_addAgent::eventAction(){
		std::cout<<"Event::TimeEvent_addConstraint "<<std::endl;
		controller_->addAgent(agent_);
	};

	TimeEvent_removeAgent::TimeEvent_removeAgent(Controller* controller, Agent* agent, double time):TimeEvent(time){
		agent_	 =agent;
		controller_=controller;
	}
	void TimeEvent_removeAgent::eventAction(){
		std::cout<<"Event::TimeEvent_removeConstraint "<<std::endl;
		controller_->removeAgent(agent_);
	};


/********************
 * Coupling Event
 */
	TimeEvent_addCoupling::TimeEvent_addCoupling(Agent* agent1,Agent* agent2,Coupling* coupling,double time):TimeEvent(time){
		agent1_	 =agent1;
		agent2_	 =agent2;
		coupling_=coupling;

	}
	void TimeEvent_addCoupling::eventAction(){
		std::cout<<"Event::TimeEvent_addCoupling "<<std::endl;
		agent1_->addCouplingAsAgent1(coupling_);
		agent2_->addCouplingAsAgent2(coupling_);
	};

	TimeEvent_removeCoupling::TimeEvent_removeCoupling(Agent* agent1,Agent* agent2,Coupling* coupling,double time):TimeEvent(time){
		agent1_	 =agent1;
		agent2_	 =agent2;
		coupling_=coupling;
	}
	void TimeEvent_removeCoupling::eventAction(){
		std::cout<<"Event::TimeEvent_removeCoupling "<<std::endl;
		agent1_->removeCoupling(coupling_);
		agent2_->removeCoupling(coupling_);
	};

/********************
 * Constraint Events
 */

	TimeEvent_addConstraint::TimeEvent_addConstraint(Agent* agent, Constraint* constraint,double time):TimeEvent(time){
		agent_	 =agent;
		constraint_=constraint;
	}
	void TimeEvent_addConstraint::eventAction(){
		std::cout<<"Event::TimeEvent_addConstraint "<<std::endl;
		agent_->addConstraint(constraint_);
	};

	TimeEvent_removeConstraint::TimeEvent_removeConstraint(Agent* agent, Constraint* constraint,double time):TimeEvent(time){
		agent_	 =agent;
		constraint_=constraint;
	}
	void TimeEvent_removeConstraint::eventAction(){
		std::cout<<"Event::TimeEvent_removeConstraint "<<std::endl;
		agent_->removeConstraint(constraint_);
	};
