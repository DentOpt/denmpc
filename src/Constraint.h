/**
 * @file    Constraint.h
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

#ifndef _CONSTRAINT_H
#define _CONSTRAINT_H

#include "Agent.h"
#include "Controller.h"
#include <vector>
#include <string>
#include <stdio.h> 		//For printf
#include <iostream>
#include "Indexing.h"

using namespace MathLib;

//Forward Declaration
class Agent;

class Constraint:public ConstraintIndex{
	friend class Controller;
protected:
	double* pc_;		//Parameters
	double* pc_init_;		//Parameters
	int 	dim_x_;
	int 	dim_u_;
	int 	dim_y_;
	int 	dim_xdes_;
	int 	dim_udes_;
	int 	dim_ydes_;
	int 	dim_p_;
	int 	dim_pc_;
	int 	dim_ineq_;
	int 	dim_eq_;
	int 	dim_l_;
	int 	dim_v_;

    std::string name_;
    int id_;

    Agent* agent_;

public: 
    
    Agent* getAgent();
    int getId(){return this->id_;};
    std::string getName(){return this->name_;};
    Constraint(int id, Agent* agent1);
    Constraint(int id);
    Constraint();

	inline void getParameter(double* _vector)		  		{for(int i=0;i<dim_pc_;i++){_vector[i]=pc_[i];}};
	inline void getParameter(std::vector<double>& _vector)	{for(int i=0;i<dim_pc_;i++){_vector[i]=pc_[i];}};
//	inline const std::vector<double>&getParameter()const	{std::vector<double> _vector (p,p+sizeof(p)/sizeof(p[0]));return _vector;};
	inline int getState_Dim()  			{return dim_x_;};
	inline int getControl_Dim()			{return dim_u_;};
	inline int getOutput_Dim() 			{return dim_y_;};
	inline int getDesiredState_Dim()  	{return dim_xdes_;};
	inline int getDesiredControl_Dim()	{return dim_udes_;};
	inline int getDesiredOutput_Dim() 	{return dim_ydes_;};
	inline int getParameter_Dim()		{return dim_pc_;};
//	inline int getConstraintParameter_Dim()	{return dim_pc;};
	inline int getInequalityConstraint_Dim(){return dim_ineq_;};
	inline int getEqualityConstraint_Dim()	{return dim_eq_;};
	inline int getStageCost_Dim() 		{return dim_l_;};
	inline int getFinalCost_Dim()		{return dim_v_;};

	//Setter Methods
	int 		setId(int _id)	 		{id_=_id;};
	std::string setName(std::string _name)	{name_=_name;};
	inline void setParameter(double* _vector)					{for(int i=0;i<dim_pc_;i++){pc_[i]=_vector[i];}};
	inline void setParameter(std::vector<double> _vector)		{for(int i=0;i<dim_pc_;i++){pc_[i]=_vector[i];}};
	inline void setInitialParameter(double* _vector)			{for(int i=0;i<dim_pc_;i++){pc_[i]=_vector[i];}};
	inline void setInitialParameter(std::vector<double> _vector){for(int i=0;i<dim_pc_;i++){pc_[i]=_vector[i];}};

/****** FUNCTIONS THAT ARE GENERATED WITH CODE GENERATION *****/
	//Stage costs
	virtual void l(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes)   {std::cout<<"Constraint "<<id_<<" "<<name_<<"::l not defined!!!"<<std::endl;}
	virtual void dldx(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes){std::cout<<"Constraint "<<id_<<" "<<name_<<"::dldx not defined!!!"<<std::endl;}
	virtual void dldu(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes){std::cout<<"Constraint "<<id_<<" "<<name_<<"::dldu not defined!!!"<<std::endl;}
	//Final costs
	virtual void v(double *out, double t, double *x, double *p, double *pc,  double *xdes)	{std::cout<<"Constraint "<<id_<<" "<<name_<<"::v not defined!!!"<<std::endl;}
	virtual void dvdx(double *out, double t, double *x, double *p, double *pc,  double *xdes){std::cout<<"Constraint "<<id_<<" "<<name_<<"::dvdx not defined!!!"<<std::endl;}
	//Equality constraints
	virtual void c(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes)					 	{std::cout<<"Constraint "<<id_<<" "<<name_<<"::c not defined!!!"<<std::endl;}
	virtual void dcdx(double **out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes)				 	{std::cout<<"Constraint "<<id_<<" "<<name_<<"::dcdx not defined!!!"<<std::endl;}
	virtual void dcdu(double **out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes)				 	{std::cout<<"Constraint "<<id_<<" "<<name_<<"::dcdu not defined!!!"<<std::endl;}
	virtual void dcdxmu(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mu_eq)	{std::cout<<"Constraint "<<id_<<" "<<name_<<"::dfdx_mu not defined!!!"<<std::endl;}
	virtual void dcdumu(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mu_eq)	{std::cout<<"Constraint "<<id_<<" "<<name_<<"::dfdu_mu not defined!!!"<<std::endl;};
	//Inequality constraints
	virtual void ci(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes)							{std::cout<<"Constraint "<<id_<<" "<<name_<<"::ci not defined!!!"<<std::endl;}
	virtual void dcidx(double **out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes)						{std::cout<<"Constraint "<<id_<<" "<<name_<<"::dcidx not defined!!!"<<std::endl;}
	virtual void dcidu(double **out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes)						{std::cout<<"Constraint "<<id_<<" "<<name_<<"::dcidu not defined!!!"<<std::endl;}
	virtual void dcidxmui(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mu_ineq)	{std::cout<<"Constraint "<<id_<<" "<<name_<<"::dcidx_mui not defined!!!"<<std::endl;}
	virtual void dcidumui(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mu_ineq)	{std::cout<<"Constraint "<<id_<<" "<<name_<<"::dcidu_mui not defined!!!"<<std::endl;}
	//Inequality with slacks
	virtual void cia(double  *out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *slack)		{std::cout<<"Constraint "<<id_<<" "<<name_<<"::cia not defined!!!"<<std::endl;}
	virtual void dciadx(double **out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *slack)	{std::cout<<"Constraint "<<id_<<" "<<name_<<"::dciadx not defined!!!"<<std::endl;}
	virtual void dciadu(double **out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *slack)	{std::cout<<"Constraint "<<id_<<" "<<name_<<"::dciadu not defined!!!"<<std::endl;}
	virtual void dciadxmui(double  *out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mu_ineq, double *slack) {std::cout<<"Constraint "<<id_<<" "<<name_<<"::dciadxmui not defined!!!"<<std::endl;}
	virtual void dciadumui(double  *out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mu_ineq, double *slack) {std::cout<<"Constraint "<<id_<<" "<<name_<<"::dciadumui not defined!!!"<<std::endl;}
	virtual void dciadamui(double  *out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mu_ineq, double *slack) {std::cout<<"Constraint "<<id_<<" "<<name_<<"::dciadamui not defined!!!"<<std::endl;}
};

#endif //_Constraint_H
