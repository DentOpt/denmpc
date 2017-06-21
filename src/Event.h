/**
 * @file    Event.h
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

#ifndef _EVENT_H
#define _EVENT_H

#include <ros/ros.h>	//For ros::
#include "std_msgs/Bool.h"

#include "dentopt_nmpc_controller/VrepInfo.h"
#include <boost/filesystem.hpp>
#include <Controller.h>
//#include "YamlInterface.h"
#include "Agent.h"
#include "Constraint.h"
#include "Controller.h"
#include <vector>		//For vector
#include <stdio.h> 		//For printf

class Event{
protected:
	bool flag_event_;
public:
	Event(){
		flag_event_=false;
	}
	virtual bool check(double time)	  {std::cout<<"Implement Event::check()"<<std::endl;};
	virtual void eventAction(){std::cout<<"Implement Event::eventAction()"<<std::endl;};
};

/****************************************************
 * ROS EVENT
 ****************************************************/

class RosEvent:public Event{
protected:
	std::string     rostopicname_;
	ros::NodeHandle node_;
	ros::Subscriber sub_;
	bool msg_content_;
public:
	bool check(double time){
		if(flag_event_){
			eventAction();
			flag_event_=false;
			return true;
		}
		flag_event_=false;
		return false;
	};
	void subCallback(const std_msgs::Bool::ConstPtr& msg){
		flag_event_=true;
		msg_content_=msg->data;
	}
	RosEvent(std::string rostopicname,bool flag_event=false){
		bool flag_event_=flag_event;
		rostopicname_=rostopicname;
		msg_content_=false;
		sub_=node_.subscribe(rostopicname_.c_str(),1,&RosEvent::subCallback,this);
	}

	virtual void eventAction(){std::cout<<"Implement RosEvent::eventAction()"<<std::endl;};
};
/********************
 * Agent Events
 */
class RosEvent_addAgent:public RosEvent{
protected:
	Controller* controller_;
	Agent*    	agent_;
public:
	//	RosEvent_addAgent(Controller* controller, Agent* agent, double time);
	//	void eventAction();
	RosEvent_addAgent(Controller* controller, Agent* agent, std::string rostopicname):RosEvent(rostopicname){
		agent_	 =agent;
		controller_=controller;
	}
	void eventAction(){
		std::cout<<"RosEvent_addAgent::eventAction() ->";
		if(msg_content_){
			std::cout<<"add agent;"<<std::endl;
			controller_->addAgent(agent_);
		}
		else{
			std::cout<<"remove agent;"<<std::endl;
			controller_->removeAgent(agent_);
		}
	};
};

class RosEvent_addCoupling:public RosEvent{
protected:
	Agent* agent1_;
	Agent* agent2_;
	Coupling* coupling_;
public:
	RosEvent_addCoupling(Agent* agent1,Agent* agent2,Coupling* coupling, std::string rostopicname):RosEvent(rostopicname){
		agent1_   =agent1;
		agent2_   =agent2;
		coupling_ =coupling;
	}
	virtual void eventAction(){
		std::cout<<"RosEvent_addCoupling::eventAction() ->";
		if(msg_content_){
			std::cout<<"add Coupling;"<<std::endl;
			agent1_->addCouplingAsAgent1(coupling_);
			agent2_->addCouplingAsAgent2(coupling_);
		}
		else{
			std::cout<<"remove Coupling;"<<std::endl;
			agent1_->removeCoupling(coupling_);
			agent2_->removeCoupling(coupling_);
		}
	}
};

class RosEvent_addConstraint:public RosEvent{
protected:
	Agent*    agent_;
	Constraint* constraint_;
public:
	RosEvent_addConstraint(Agent* agent, Constraint* constraint, std::string rostopicname):RosEvent(rostopicname){
		agent_=agent;
		constraint_=constraint;
	}
	virtual void eventAction(){
		std::cout<<"RosEvent_addConstraint::eventAction() ->";
			if(msg_content_){
				std::cout<<"add Constraint;"<<std::endl;
				agent_->addConstraint(constraint_);
			}
			else{
				std::cout<<"remove Constraint;"<<std::endl;
				agent_->removeConstraint(constraint_);
			}
	}
};





/****************************************************
 * Time EVENT
 ****************************************************/

class TimeEvent:public Event{
protected:
	double time_;
public:
	TimeEvent(double time=0,bool flag_event=false){
		time_=time;
		flag_event_=flag_event;
	}
	bool check(double time){
		if((flag_event_==false)&&(time>time_)){
			flag_event_=true;
			eventAction();
			return true;
		}
		else{
			return false;
		}
	};
	virtual void eventAction(){std::cout<<"Implement TimeEvent::eventAction()"<<std::endl;};
};

/********************
 * Parameter Events
 */
class TimeEvent_changeParameters:public TimeEvent{
protected:
	double*     parameters_;
	Agent*      agent_;
	Constraint* constraint_;
	Coupling*   coupling_;
public:
	TimeEvent_changeParameters(Agent* agent, double* p,double time);
	TimeEvent_changeParameters(Constraint* constraint, double* pc,double time);
	TimeEvent_changeParameters(Coupling* coupling, double* pc,double time);
	virtual void eventAction();
};
/********************
 * Agent Events
 */
class TimeEvent_addAgent:public TimeEvent{
protected:
	Controller* controller_;
	Agent*    	agent_;
public:
	TimeEvent_addAgent(Controller* controller, Agent* agent, double time);
	void eventAction();
};
class TimeEvent_removeAgent:public TimeEvent{
protected:
	Controller* controller_;
	Agent*    	agent_;
public:
	TimeEvent_removeAgent(Controller* controller, Agent* agent, double time);
	void eventAction();
};


/********************
 * Coupling Event
 */
class TimeEvent_addCoupling:public TimeEvent{
protected:
	Agent* agent1_;
	Agent* agent2_;
	Coupling* coupling_;
public:
	TimeEvent_addCoupling(Agent* agent1,Agent* agent2,Coupling* coupling,double time);
	virtual void eventAction();
};
class TimeEvent_removeCoupling:public TimeEvent{
protected:
	Agent*    agent1_;
	Agent*    agent2_;
	Coupling* coupling_;
public:
	TimeEvent_removeCoupling(Agent* agent1,Agent* agent2,Coupling* coupling,double time);
	void eventAction();
};


/********************
 * Constraint Events
 */
class TimeEvent_addConstraint:public TimeEvent{
protected:
	Agent*    agent_;
	Constraint* constraint_;
public:
	TimeEvent_addConstraint(Agent* agent, Constraint* constraint,double time);
	virtual void eventAction();
};
class TimeEvent_removeConstraint:public TimeEvent{
protected:
	Agent*    agent_;
	Constraint* constraint_;
public:
	TimeEvent_removeConstraint(Agent* agent, Constraint* constraint,double time);
	void eventAction();
};


#endif //_Event_H
