/**
 * @file    Coupling.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Coupling Container
 *
 * This can be used to implement couplings between agents
 * It covers stage costs, final costs, inequality constraints, equality constraints
 * E.g. Collision Avoidance constraints
 */

#include "Coupling.h"

/**
 * Coupling implementation
 */

Coupling::Coupling(int id,Agent* _agent1, Agent* _agent2){
	this->id_=id;
	this->name_="general Coupling";
	agent1_=_agent1;
	agent2_=_agent2;
//	agent1->addCoupling1(this);
//	agent2->addCoupling2(this);
}
