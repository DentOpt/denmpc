/**
 * @file    Constraint.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Constraint Container
 *
 * This can be used to implement constraints on system states,
 * controls, etc. it covers stage costs, final costs, inequality constraints, equality constraints
 * E.g. Orientation Constraints
 */


#include "Constraint.h"

/**
 * Constraint implementation
 */

Agent* Constraint::getAgent() {
	return agent_;
}
Constraint::Constraint(){
	std::cout<<"Constraint::Constraint()"<<std::endl;
};
Constraint::Constraint(int id){
	this->id_=id;
	this->name_="general Constraint";
}
Constraint::Constraint(int id,Agent* _agent){
	this->id_=id;
	this->name_="general Constraint";
	agent_=_agent;
	agent_->addConstraint(this);

}
