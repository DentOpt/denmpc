/**
 * @file    Coupling.h
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

#ifndef _COUPLING_H
#define _COUPLING_H

#include "Indexing.h"
#include "Agent.h"
#include "Constraint.h"
#include <vector>
#include <string>
#include <stdio.h> 		//For printf
#include <iostream>

using namespace MathLib;

//Forward Declaration
class Agent;
class Constraint;

class Coupling : public CouplingIndex{
	friend class Controller;
protected:
	int id_;
	std::string name_;
	double* pc_;		//Parameters
	double* pc_init_;		//Parameters
	//State Dimensions
	int 	dim_x1_;
	int 	dim_u1_;
	int 	dim_y1_;
	int 	dim_xdes1_;
	int 	dim_udes1_;
	int 	dim_ydes1_;
	int 	dim_p1_;
	int 	dim_x2_;
	int 	dim_u2_;
	int 	dim_y2_;
	int 	dim_xdes2_;
	int 	dim_udes2_;
	int 	dim_ydes2_;
	int 	dim_p2_;
	int 	dim_pc_;

	int dim_p_;
	int dim_ineq_;
	int dim_eq_;
	int dim_l_;
	int dim_v_;
    Agent* agent1_;
    Agent* agent2_;

public: 
    inline void setInitialParameter(double* _vector){for(int i=0;i<dim_pc_;i++){pc_init_[i]=_vector[i];}};
    inline void setParameter(double* _vector){for(int i=0;i<dim_pc_;i++){pc_[i]=_vector[i];}};

    inline int getInequalityConstraint_Dim(){return dim_ineq_;};
	inline int getEqualityConstraint_Dim()	{return dim_eq_;};
	inline int getStageCost_Dim() 			{return dim_l_;};
	inline int getFinalCost_Dim()			{return dim_v_;};
//	inline int getConstraintParameter_Dim()	{return dim_pc;};
	inline void getInitialParameter(double* _vector)		  		{for(int i=0;i<dim_pc_;i++){_vector[i]=pc_init_[i];}};
	inline void getInitialParameter(std::vector<double>& _vector)	{for(int i=0;i<dim_pc_;i++){_vector[i]=pc_init_[i];}};
	inline void getParameter(double* _vector)		  		{for(int i=0;i<dim_pc_;i++){_vector[i]=pc_[i];}};
	inline void getParameter(std::vector<double>& _vector)	{for(int i=0;i<dim_pc_;i++){_vector[i]=pc_[i];}};
	inline int getParameter_Dim()		{return dim_pc_;};
//    Agent* getAgent1();
//    Agent* getAgent2();
    Agent* getAgent1() {return agent1_;}
    Agent* getAgent2() {return agent2_;}
    void setAgent1(Agent* _agent1) {agent1_=_agent1;}
    void setAgent2(Agent* _agent2) {agent2_=_agent2;}

    int getId(){return this->id_;};
    std::string getName(){return this->name_;};
    Coupling(){};
	Coupling(int _id){id_=_id;};
    Coupling(int id_, Agent* agent1, Agent* agent2);
	//Stage costs
	virtual void l(double *out, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2)   {std::cout<<"Coupling "<<id_<<" "<<name_<<"::l not defined!!!"<<std::endl;}
	virtual void dldx(double *dldx1, double *dldx2, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2){std::cout<<"Coupling "<<id_<<" "<<name_<<"::dldx not defined!!!"<<std::endl;}
	virtual void dldu(double *dldu1, double *dldu2, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2){std::cout<<"Coupling "<<id_<<" "<<name_<<"::dldu not defined!!!"<<std::endl;}
	//Final costs
	virtual void v(double *out, double t, double *x1,double *x2, double *p1,double *p2, double *pc,  double *xdes1, double *xdes2)	{std::cout<<"Coupling "<<id_<<" "<<name_<<"::v not defined!!!"<<std::endl;}
	virtual void dvdx(double *dvdx1, double *dvdx2, double t, double *x1,double *x2, double *p1,double *p2, double *pc,  double *xdes1, double *xdes2){std::cout<<"Coupling "<<id_<<" "<<name_<<"::dvdx not defined!!!"<<std::endl;}
	//Equality constraints
	virtual void c(double *out, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2)					 	{std::cout<<"Coupling "<<id_<<" "<<name_<<"::c not defined!!!"<<std::endl;}
//	virtual void dcdx(double **out, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2)				 	{std::cout<<"Coupling "<<id_<<" "<<name_<<"::dcdx not defined!!!"<<std::endl;}
//	virtual void dcdu(double **out, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2)				 	{std::cout<<"Coupling "<<id_<<" "<<name_<<"::dcdu not defined!!!"<<std::endl;}
	virtual void dcdxmu(double *dcdx1mu, double *dcdx2mu, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2, double *mu_eq)	{std::cout<<"Coupling "<<id_<<" "<<name_<<"::dfdx_mu not defined!!!"<<std::endl;}
	virtual void dcdumu(double *dcdu1mu, double *dcdu2mu, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2, double *mu_eq)	{std::cout<<"Coupling "<<id_<<" "<<name_<<"::dfdu_mu not defined!!!"<<std::endl;};
	//Inequality constraints
	virtual void ci(double *out, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2)							{std::cout<<"Coupling "<<id_<<" "<<name_<<"::ci not defined!!!"<<std::endl;}
//	virtual void dcidx(double **out, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2)						{std::cout<<"Coupling "<<id_<<" "<<name_<<"::dcidx not defined!!!"<<std::endl;}
//	virtual void dcidu(double **out, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2)						{std::cout<<"Coupling "<<id_<<" "<<name_<<"::dcidu not defined!!!"<<std::endl;}
	virtual void dcidxmui(double *dcidx1mui, double *dcidx2mui, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2, double *mu_ineq)	{std::cout<<"Coupling "<<id_<<" "<<name_<<"::dcidx_mui not defined!!!"<<std::endl;}
	virtual void dcidumui(double *dcidu1mui, double *dcidu2mui, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2, double *mu_ineq)	{std::cout<<"Coupling "<<id_<<" "<<name_<<"::dcidu_mui not defined!!!"<<std::endl;}
	//Inequality with slacks
	virtual void cia(double  *out,double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2, double *slack)		{std::cout<<"Coupling "<<id_<<" "<<name_<<"::cia not defined!!!"<<std::endl;}
//	virtual void dciadx(double **out,double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2, double *slack)	{std::cout<<"Coupling "<<id_<<" "<<name_<<"::dciadx not defined!!!"<<std::endl;}
//	virtual void dciadu(double **out,double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2, double *slack)	{std::cout<<"Coupling "<<id_<<" "<<name_<<"::dciadu not defined!!!"<<std::endl;}
	virtual void dciadxmui(double *dciadx1mui, double *dciadx2mui, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2, double *mu_ineq, double *slack) {std::cout<<"Coupling "<<id_<<" "<<name_<<"::dciadxmui not defined!!!"<<std::endl;}
	virtual void dciadumui(double *dciadu1mui, double *dciadu2mui, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2, double *mu_ineq, double *slack) {std::cout<<"Coupling "<<id_<<" "<<name_<<"::dciadumui not defined!!!"<<std::endl;}
	virtual void dciadamui(double *dciadamui, double t, double *x1,double *x2, double *u1,double *u2, double *p1,double *p2, double *pc,  double *xdes1,double *xdes2, double *udes1,double *udes2, double *mu_ineq, double *slack) {std::cout<<"Coupling "<<id_<<" "<<name_<<"::dciadamui not defined!!!"<<std::endl;}
};

#endif //_Constraint_H
