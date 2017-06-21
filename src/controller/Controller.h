/**
 * @file    Controller.h
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Controller Container
 *
 *	Controller represents a Container Class for MPC-Controllers.
 *	It generates the functions required for optimization, such as
 *	the Hamiltonian and its derivatives.
 *	The generation is based on a concatenation of all controlled agents, constraints and couplings.
 *	The concatenation is in respect to the chosen constraint handling:
 *	1. Auxiliary Variable Method as implemented by Ohtsuka
 *	where the inequality is transformed to an equality constraint with the help of slack variables
 *	Ohtsuka, T., “A Continuation/GMRES Method for Fast Computation of Nonlinear Receding Horizon Control,” Automatica, Vol. 40, No. 4, Apr. 2004, pp. 563-574.
 *	2. Simple Primal Barrier Method
 *	where the inequality is transformed into stage cost with the help of a logarithmic barrier function.
 */

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "Agent.h"
#include "Constraint.h"
#include "Coupling.h"
#include <time.h>
#include <fstream>
#include <vector>
#include "std_msgs/Float32.h"

using namespace MathLib;

//Forward Declarations
class Agent;
class Constraint;
class Coupling;

/* ===============================================
 * Controller is concatenating Hamiltonian
 * according to coptimization variables
 * optvar=[	agent1_u,			...,	agentN_u, \
 * 		agent1_lambda,		...,	agentN_lambda,\
 *		agent1c1_lambda,	...,	agent1cM_lambda,
 *		...,
 *		agentNc1_lambda,	...,	agentNcM_lambda,
 *		constraint1_lambda,	...,	constraintN_lambda,
 *
 *
 *		]
 * x=[agent1_x,agent2_x,...,agentN_x]...
 * =============================================== */

//Defining numbers for Constraint handling methods
#define METHOD_PRIMALBARRIER     0
#define METHOD_AUXILIARYVARIABLE 1
#define METHOD_ACTIVESET         2
#define METHOD_EXTERIORPENALTY   3

class Controller{
	friend class Cmscgmres;
protected:
	int id_;

	//Constraint handling factors
	double primalbarrierfactor_;
	double exteriorpenaltyfactor_;
	double lambda_init_;
	double mu_init_;
	double mui_init_;
	double slack_init_;
	double slack_penalty_init_;

	//Lists and Pointers
	std::vector<Agent*> agentlist_;
	Agent* 		tmp_agent_ptr_;
	Constraint* tmp_constraint_ptr_;
	Coupling*   tmp_coupling_ptr_;

	//Dimensions
	int dim_optvar_conc_;	//Concatenation of optimization variables
	int dim_x_conc_;			//Concatenation of states
	int dim_u_conc_;			//Concatenation of controls
	int dim_xdes_conc_;		//Concatenation of desired states
	int dim_udes_conc_;		//Concatenation of desired controls
	int dim_d_conc_;			//Concatenation of disturbance
	int dim_p_conc_;			//Concatenation of parameters
	int dim_lambda_conc_;	//Concatenation of state lagrange mutlipliers
	int dim_eqcon_conc_;		//Concatenation of equality lagrange multipliers
	int dim_ineqcon_conc_;	//Concatenation of inequality lagrange multipliers

	//Arrays
	double* optvar_conc_;	//Concatenation of optimization variables
	double* x_conc_;
	double* u_conc_;
	double* d_conc_;
	double* p_conc_;
	double* udes_conc_;
	double* xdes_conc_;
	double* lambda_conc_;
	double* mu_conc_;
	double* mui_conc_;
	double* slack_conc_;
	double* slack_penalty_conc_;

	int constrainthandlingmethod_;
	bool* workingset_;

	//Flags
	bool flag_memoryalreadyallocated_; //True if memory is allocated
	bool flag_show_controllertrace_;	  //Show interior vector and matrix values of controller
	bool flag_show_controllerinfo_;	  //Show controller info
	bool flag_show_controllerstates_;  //Show controller states
	bool flag_save_controllerlog_;	  //Save controllers

	//Log
	std::string 	  log_filename_;
	std::stringstream log_stringstream_;
	std::ofstream 	  log_file_;
	ros::NodeHandle   node_;
	ros::Publisher 	  comp_time_publisher_;

public:

	void startAgents();

	void setInitialEqualityConstraintLagrangeMultiplier	 (double _mu_init)	  {mu_init_=_mu_init;}
	void setInitialInequalityConstraintLagrangeMultiplier(double _mui_init)   {mui_init_=_mui_init;}
	void setInitialAuxiliarySlackVariables 			     (double _slack_init) {slack_init_=_slack_init;}

	void activateInfo_ControllerTrace(){flag_show_controllertrace_=true;}
	void activateInfo_Controller(){flag_show_controllerinfo_=true;}
	void activateInfo_ControllerStates(){flag_show_controllerstates_=true;}
	void startLogging2File(){flag_save_controllerlog_=true;}
	void deactivateInfo_ControllerTrace(){flag_show_controllertrace_=false;}
	void deactivateInfo_Controller(){flag_show_controllerinfo_=false;}
	void deactivateInfo_ControllerStates(){flag_show_controllerstates_=false;}
	void stopLogging2File(){flag_save_controllerlog_=false;}

	int getXdesConc_Dim(){return dim_xdes_conc_;}
	void setXdesConc(double* vector){
		MathLib::mmov(xdes_conc_,dim_xdes_conc_,vector);
	}
	void getXdesConc(double* vector){
		MathLib::mmov(vector,dim_xdes_conc_,xdes_conc_);
	}
	int getXConc_Dim(){return dim_x_conc_;}
	void setXConc(double* vector){
		MathLib::mmov(x_conc_,dim_x_conc_,vector);
	}
	void getXConc(double* vector){
		MathLib::mmov(vector,dim_x_conc_,x_conc_);
	}


	//Constructor
	Controller(std::vector<Agent*> _listofagents, int _id);
	//Destructor
	~Controller();
	//Initialization: Concatenation of the Vectors
	void initConcatenation();
	//	//get initial values and set them to the arrays
	//	void initArrayValues();
	//Get Measurments from Agents
	void getMeasurements();
	//Compute action
	virtual void computeAction(double _time){std::cout<<"void Controller::computeAction()"<<std::endl;};
	//Apply computed action
	void applyAction();
	//Test initial agent and constraint values
	void test();

	void addAgent(Agent* agent);
	Agent* getAgent(int index){return agentlist_[index];};
	int getAgent_Dim(){return agentlist_.size();};
	void removeAgent(Agent* agent){agentlist_.erase(std::remove(agentlist_.begin(),  agentlist_.end(),  agent),agentlist_.end());};

	virtual void init(){initConcatenation();};

	void integrateStateExplicitEuler(double time,double integrationstep){
		double dx_tmp[dim_x_conc_];
		f(dx_tmp,time,x_conc_,u_conc_,d_conc_,p_conc_);
		for(int i=0;i<dim_x_conc_;i++){
			x_conc_[i]+=integrationstep*dx_tmp[i];
		}
	}

	void calculateActuation(double time);
	// calculate plant output from measurement with estimator	-> Call Estimator/Observer
	void calculateState();
	//Compute active set
	void setActiveSetWorkingSet(double* mui);
	/*=============================================
	 * Systemfunction Composition
	 *============================================= */
	//	void dHduPrimalBarrier(double  *out,double t, double *x, double *u, double *p){
	//		double* xdes	=p	 +p_conc_dim;
	//		double* udes	=xdes+xdes_conc_dim;
	//		double* lambda	=x	 +dim_x_conc_;
	//		double* mu		=u	 +dim_u_conc_;
	//		double* mui		=mu	 +eqcon_conc_dim_;
	//		dHdu  	(out,						t,x,u,p,xdes,udes,lambda,mu,mui);
	//		dcdumu	(out+dim_u_conc_,			t,x,u,p,xdes,udes,mu);
	//		dcidumui(out+dim_u_conc_+eqcon_conc_dim_,t,x,u,p,xdes,udes,mui);
	//
	//	}
	//	void dHdu(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes, double* lambda, double* mu, double* mui);
	//	void dHdx(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes, double* lambda, double* mu, double* mui);
	/***************************************************************************************
	 *	Hamiltonian=l+lambda*f+mui*ci+mu*c
	 *	=>dHdu=dldu+lambda*dfdu+mui*dcidu+mu*dcdu
	 *	0=dHdu for optimality
	 ***************************************************************************************/
	inline void dHdu(double  *out,double t, double *x, double *optcon, double *d, double *p, double *xdes, double *udes, double* lambda){
		switch(constrainthandlingmethod_){
		case METHOD_PRIMALBARRIER:{
			double* tmp_ptr_u  =optcon;
			double* tmp_ptr_mu =tmp_ptr_u +dim_u_conc_;
			//		//linp=[x,optvar] //Primal Barrier => linp=[x,u,mu,mui]
			//		double* x=linp;
			//		double* u=linp +dim_x_conc_;
			//		double* mu=linp+dim_x_conc_+dim_u_conc_;
			//		this->minusdHdxPrimalBarrier(lambdaprime,t,x,u,p_conc_,xdes,udes,lambda,mu);
			dHduPrimalBarrier(out,t,x,tmp_ptr_u,d,p,xdes,udes,lambda,tmp_ptr_mu);
			break;
		}
		case METHOD_AUXILIARYVARIABLE:{
			double* tmp_ptr_u    =optcon;
			double* tmp_ptr_mu   =tmp_ptr_u   +dim_u_conc_;
			double* tmp_ptr_mui  =tmp_ptr_mu  +dim_eqcon_conc_;
			double* tmp_ptr_slack=tmp_ptr_mui +dim_ineqcon_conc_;
			//		//linp=[x,optvar] //Primal Barrier => linp=[x,u,mu,mui,slack]
			//		double* x=linp;
			//		double* u=linp +dim_x_conc_;
			//		double* mu=linp+dim_x_conc_+dim_u_conc_;
			//		this->minusdHdxPrimalBarrier(lambdaprime,t,x,u,p_conc_,xdes,udes,lambda,mu);
			dHduAuxiliaryVariable(out,t,x,tmp_ptr_u,d,p,xdes,udes,lambda,tmp_ptr_mu,tmp_ptr_mui,tmp_ptr_slack);
			break;
		}
		case METHOD_ACTIVESET:{
			double* tmp_ptr_u    =optcon;
			double* tmp_ptr_mu   =tmp_ptr_u   +dim_u_conc_;
			double* tmp_ptr_mui  =tmp_ptr_mu  +dim_eqcon_conc_;
			double* tmp_ptr_slack=tmp_ptr_mui +dim_ineqcon_conc_;
			//		//linp=[x,optvar] //Primal Barrier => linp=[x,u,mu,mui,slack]
			//		double* x=linp;
			//		double* u=linp +dim_x_conc_;
			//		double* mu=linp+dim_x_conc_+dim_u_conc_;
			//		this->minusdHdxPrimalBarrier(lambdaprime,t,x,u,p_conc_,xdes,udes,lambda,mu);
			dHduActiveSet(out,t,x,tmp_ptr_u,d,p,xdes,udes,lambda,tmp_ptr_mu,tmp_ptr_mui);
			break;
		}
		case METHOD_EXTERIORPENALTY:{
			double* tmp_ptr_u    =optcon;
			double* tmp_ptr_mu   =tmp_ptr_u   +dim_u_conc_;
			double* tmp_ptr_mui  =tmp_ptr_mu  +dim_eqcon_conc_;
			double* tmp_ptr_slack=tmp_ptr_mui +dim_ineqcon_conc_;
			//		//linp=[x,optvar] //Primal Barrier => linp=[x,u,mu,mui,slack]
			//		double* x=linp;
			//		double* u=linp +dim_x_conc_;
			//		double* mu=linp+dim_x_conc_+dim_u_conc_;
			//		this->minusdHdxPrimalBarrier(lambdaprime,t,x,u,p_conc_,xdes,udes,lambda,mu);
			dHduExteriorPenalty(out,t,x,tmp_ptr_u,d,p,xdes,udes,lambda,tmp_ptr_mu,tmp_ptr_mui);
			break;
		}
		}
	};

	/***************************************************************************************
	 *	Hamiltonian=l+lambda*f+mui*ci+mu*c
	 *	=>dHdx=dldx+lambda*dfdx+mui*dcidx+mu*dcdx
	 *	dlambda=-dHdx for optimality
	 ***************************************************************************************/
	inline void minusdHdx(double  *out,double t, double *x, double *optcon, double *d, double *p, double *xdes, double *udes, double* lambda){

		switch(constrainthandlingmethod_){
		case METHOD_PRIMALBARRIER:{
			double* tmp_ptr_u  =optcon;
			double* tmp_ptr_mu =tmp_ptr_u +dim_u_conc_;
			//		//linp=[x,optvar] //Primal Barrier => linp=[x,u,mu,mui]
			//		double* x=linp;
			//		double* u=linp +dim_x_conc_;
			//		double* mu=linp+dim_x_conc_+dim_u_conc_;
			//		this->minusdHdxPrimalBarrier(lambdaprime,t,x,u,p_conc_,xdes,udes,lambda,mu);
			minusdHdxPrimalBarrier(out,t,x,tmp_ptr_u,d,p,xdes,udes,lambda,tmp_ptr_mu);
			break;
		}
		case METHOD_AUXILIARYVARIABLE:{
			double* tmp_ptr_u    =optcon;
			double* tmp_ptr_mu   =tmp_ptr_u   +dim_u_conc_;
			double* tmp_ptr_mui  =tmp_ptr_mu  +dim_eqcon_conc_;
			double* tmp_ptr_slack=tmp_ptr_mui +dim_ineqcon_conc_;
			//		//linp=[x,optvar] //Primal Barrier => linp=[x,u,mu,mui,slack]
			//		double* x=linp;
			//		double* u=linp +dim_x_conc_;
			//		double* mu=linp+dim_x_conc_+dim_u_conc_;
			//		this->minusdHdxPrimalBarrier(lambdaprime,t,x,u,p_conc_,xdes,udes,lambda,mu);
			minusdHdxAuxiliaryVariable(out,t,x,tmp_ptr_u,d,p,xdes,udes,lambda,tmp_ptr_mu,tmp_ptr_mui,tmp_ptr_slack);
			break;
		}
		case METHOD_ACTIVESET:{
			double* tmp_ptr_u    =optcon;
			double* tmp_ptr_mu   =tmp_ptr_u   +dim_u_conc_;
			double* tmp_ptr_mui  =tmp_ptr_mu  +dim_eqcon_conc_;
			double* tmp_ptr_slack=tmp_ptr_mui +dim_ineqcon_conc_;
			//		//linp=[x,optvar] //Primal Barrier => linp=[x,u,mu,mui,slack]
			//		double* x=linp;
			//		double* u=linp +dim_x_conc_;
			//		double* mu=linp+dim_x_conc_+dim_u_conc_;
			//		this->minusdHdxPrimalBarrier(lambdaprime,t,x,u,p_conc_,xdes,udes,lambda,mu);
			minusdHdxActiveSet(out,t,x,tmp_ptr_u,d,p,xdes,udes,lambda,tmp_ptr_mu,tmp_ptr_mui);
			break;
		}
		case METHOD_EXTERIORPENALTY:{
			double* tmp_ptr_u    =optcon;
			double* tmp_ptr_mu   =tmp_ptr_u   +dim_u_conc_;
			double* tmp_ptr_mui  =tmp_ptr_mu  +dim_eqcon_conc_;
			double* tmp_ptr_slack=tmp_ptr_mui +dim_ineqcon_conc_;
			//		//linp=[x,optvar] //Primal Barrier => linp=[x,u,mu,mui,slack]
			//		double* x=linp;
			//		double* u=linp +dim_x_conc_;
			//		double* mu=linp+dim_x_conc_+dim_u_conc_;
			//		this->minusdHdxPrimalBarrier(lambdaprime,t,x,u,p_conc_,xdes,udes,lambda,mu);
			minusdHdxExteriorPenalty(out,t,x,tmp_ptr_u,d,p,xdes,udes,lambda,tmp_ptr_mu,tmp_ptr_mui);
			break;
		}
		}

	};

	/***************************************************************************************
	 *	Hamiltonian=l+lambda*f+mui*ci+mu*c
	 *	=>dHdu=dldu+lambda*dfdu+mui*dcidu+mu*dcdu
	 *	0=dHdu for optimality
	 ***************************************************************************************/
	inline void dHduPrimalBarrier(double  *out,double t, double *x, double *u, double *d, double *p, double *xdes, double *udes, double* lambda, double* mu){
		double tmp_u[dim_u_conc_];
		memset(tmp_u, 0, dim_u_conc_*sizeof (double));
		//		for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;}

		//Get derivative of stage cost dldu
		dldu		(out,t,x,u,p,xdes,udes);
		//Get derivative of system function dfdx*lambda
		dfdulambda	(tmp_u,t,x,u,d,p,lambda);
		for(int it_control=0; it_control<dim_u_conc_; it_control++){out[it_control]+=tmp_u[it_control];}
		//Check if equalities exist
		if(dim_eqcon_conc_){
			//Get derivative of equality constraint dcidu
			dcdumu		(tmp_u,t,x,u,p,xdes,udes,mu);
			for(int it_control=0; it_control<dim_u_conc_; it_control++){out[it_control]+=tmp_u[it_control];}
			c(out+dim_u_conc_,t,x,u,p,xdes,udes);
		}
		//Check if inequalities exist
		if(dim_ineqcon_conc_){
			//Create primal barrier derivative -tau*ci/dcidu
			double tmp_ci[dim_ineqcon_conc_];
			//Get constraint functions
			ci(tmp_ci,t,x,u,p,xdes,udes);
			for(int it_constraint=0; it_constraint<dim_ineqcon_conc_; it_constraint++){tmp_ci[it_constraint]=-primalbarrierfactor_/tmp_ci[it_constraint];}
			//Get derivative of inequality constraint with calculated factors
			dcidumui	(tmp_u,t,x,u,p,xdes,udes,tmp_ci);
			for(int it_control=0; it_control<dim_u_conc_; it_control++){out[it_control]+=tmp_u[it_control];}
		}
	};
	inline void minusdHdxPrimalBarrier(double  *out,double t, double *x, double *u, double *d, double *p, double *xdes, double *udes, double* lambda, double* mu){
		double tmp_x[dim_x_conc_];
		memset(tmp_x, 0, dim_x_conc_*sizeof (double));
		//		for(int it_control=0; it_control<dim_x_conc_; it_control++){tmp_x[it_control]=0;}

		dldx		(out,t,x,u,p,xdes,udes);
		dfdxlambda	(tmp_x,t,x,u,d,p,lambda);
		for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]+=tmp_x[it_state];}
		if(dim_eqcon_conc_){
			dcdxmu		(tmp_x,t,x,u,p,xdes,udes,mu);
			for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]+=tmp_x[it_state];}
		}
		//Check if inequalities exist
		if(dim_ineqcon_conc_){
			//Create primal barrier derivative -tau*ci/dcidu
			double tmp_ci[dim_ineqcon_conc_];
			//Get constraint functions
			ci(tmp_ci,t,x,u,p,xdes,udes);
			for(int it_constraint=0; it_constraint<dim_ineqcon_conc_; it_constraint++){tmp_ci[it_constraint]=-primalbarrierfactor_/tmp_ci[it_constraint];}
			//Get derivative of inequality constraint with calculated factors
			dcidxmui	(tmp_x,t,x,u,p,xdes,udes,tmp_ci);
			for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]+=tmp_x[it_state];}
		}
		//Negate dHdx
		for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]=-out[it_state];}
	};
	/***************************************************************************************
	 *	Hamiltonian=l+lambda*f+mui*ci+mu*(c+slack^2)-k*a
	 *	=>dHdu=[dldu+lambda*dfdu+mui*dcidu+mu*dcdu;ci;cia;dciada
	 *	0=dHdu for optimality
	 ***************************************************************************************/
	inline void dHduAuxiliaryVariable(double  *out,double t, double *x, double *u, double *d, double *p, double *xdes, double *udes, double* lambda, double* mu, double* mui, double* slack){
		double tmp_u[dim_u_conc_];
		memset(tmp_u, 0, dim_u_conc_*sizeof (double));
		//		for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;}

		//Get derivative of stage cost dldu
		dldu		(out,t,x,u,p,xdes,udes);
		//Get derivative of system function dfdx*lambda
		dfdulambda	(tmp_u,t,x,u,d,p,lambda);
		for(int it_control=0; it_control<dim_u_conc_; it_control++){out[it_control]+=tmp_u[it_control];}
		//Check if equalities exist
		if(dim_eqcon_conc_){
			//Get derivative of equality constraint dcidu
			dcdumu		(tmp_u,t,x,u,p,xdes,udes,mu);
			for(int it_control=0; it_control<dim_u_conc_; it_control++){out[it_control]+=tmp_u[it_control];}
			c(out+dim_u_conc_,t,x,u,p,xdes,udes);
		}
		//Check if inequalities exist
		if(dim_ineqcon_conc_){
			//Get derivative of inequality constraint with calculated factors
			double tmp_ci[dim_ineqcon_conc_];
			double* out_mui=out+dim_u_conc_+dim_eqcon_conc_;
			double* out_slack=out_mui+dim_ineqcon_conc_;

			dciadumui	(tmp_ci,t,x,u,p,xdes,udes,mui,slack);
			for(int it_control=0; it_control<dim_u_conc_; it_control++){out[it_control]+=tmp_u[it_control];}
			//d mu*cia dmu=cia
			cia			(out_mui,t,x,u,p,xdes,udes,slack);
			//d mu*cia da=mui*dciada
			dciadamui	(tmp_ci,t,x,u,p,xdes,udes,mui,slack);
			for(int it_slack=0; it_slack<dim_u_conc_; it_slack++){out_slack[it_slack]=tmp_ci[it_slack]-slack_penalty_conc_[it_slack]*slack[it_slack];}
		}
	};
	/***************************************************************************************
	 *	Hamiltonian=l+lambda*f+mui*ci+mu*(c+slack^2)-k*a
	 *	=>minusdHdx=-[dldx+lambda*dfdx+mui*dcidx+mu*dcdx]
	 ***************************************************************************************/
	inline void minusdHdxAuxiliaryVariable(double  *out,double t, double *x, double *u, double *d, double *p, double *xdes, double *udes, double* lambda, double* mu, double* mui, double* slack){
		double tmp_x[dim_x_conc_];
		memset(tmp_x, 0, dim_x_conc_*sizeof (double));
		//		for(int it_control=0; it_control<dim_x_conc_; it_control++){tmp_x[it_control]=0;}

		dldx		(out,t,x,u,p,xdes,udes);
		dfdxlambda	(tmp_x,t,x,u,d,p,lambda);
		for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]+=tmp_x[it_state];}
		if(dim_eqcon_conc_){
			dcdxmu		(tmp_x,t,x,u,p,xdes,udes,mu);
			for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]+=tmp_x[it_state];}
		}
		//Check if inequalities exist
		if(dim_ineqcon_conc_){
			//Create primal barrier derivative -tau*ci/dcidu
			double tmp_ci[dim_ineqcon_conc_];
			//Get derivative of inequality constraint with calculated factors
			dciadxmui	(tmp_x,t,x,u,p,xdes,udes,mui,slack);
			for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]+=tmp_x[it_state];}
		}
		//Negate dHdx
		for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]=-out[it_state];}
	};
	/***************************************************************************************
	 *	Hamiltonian=l+lambda*f+mui*ci+mu*(c+slack^2)-k*a
	 *	=>dHdu=[dldu+lambda*dfdu+mui*dcidu+mu*dcdu;ci;cia;dciada
	 *	0=dHdu for optimality
	 ***************************************************************************************/
	inline void dHduActiveSet(double  *out,double t, double *x, double *u, double *d, double *p, double *xdes, double *udes, double* lambda, double* mu, double* mui){
		double tmp_u[dim_u_conc_];
		memset(tmp_u, 0, dim_u_conc_*sizeof (double));
		//		for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;}

		//Get derivative of stage cost dldu
		dldu		(out,t,x,u,p,xdes,udes);
		//Get derivative of system function dfdx*lambda
		dfdulambda	(tmp_u,t,x,u,d,p,lambda);
		for(int it_control=0; it_control<dim_u_conc_; it_control++){out[it_control]+=tmp_u[it_control];}
		//Check if equalities exist
		if(dim_eqcon_conc_){
			//Get derivative of equality constraint dcidu
			dcdumu		(tmp_u,t,x,u,p,xdes,udes,mu);
			for(int it_control=0; it_control<dim_u_conc_; it_control++){out[it_control]+=tmp_u[it_control];}
			c(out+dim_u_conc_,t,x,u,p,xdes,udes);
		}
		//Check if inequalities exist
		if(dim_ineqcon_conc_){
			//Create primal barrier derivative -tau*ci/dcidu
			double tmp_ci[dim_ineqcon_conc_];
			ci(tmp_ci,t,x,u,p,xdes,udes);
			//Get constraint functions
			for(int i=0;i<dim_ineqcon_conc_;i++){
				if(workingset_[i]){
					tmp_ci[i]=0;
				}
			}
			//Get derivative of inequality constraint with calculated factors
			dcidumui	(tmp_u,t,x,u,p,xdes,udes,tmp_ci);
			for(int it_control=0; it_control<dim_u_conc_; it_control++){out[it_control]+=tmp_u[it_control];}
		}
	};
	/***************************************************************************************
	 *	Hamiltonian=l+lambda*f+mui*ci+mu*(c+slack^2)-k*a
	 *	=>minusdHdx=-[dldx+lambda*dfdx+mui*dcidx+mu*dcdx]
	 ***************************************************************************************/
	inline void minusdHdxActiveSet(double  *out,double t, double *x, double *u, double *d, double *p, double *xdes, double *udes, double* lambda, double* mu, double* mui){
		double tmp_x[dim_x_conc_];
		memset(tmp_x, 0, dim_x_conc_*sizeof (double));
		//		for(int it_control=0; it_control<dim_x_conc_; it_control++){tmp_x[it_control]=0;}

		dldx		(out,t,x,u,p,xdes,udes);
		dfdxlambda	(tmp_x,t,x,u,d,p,lambda);
		for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]+=tmp_x[it_state];}
		if(dim_eqcon_conc_){
			dcdxmu		(tmp_x,t,x,u,p,xdes,udes,mu);
			for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]+=tmp_x[it_state];}
		}
		//Check if inequalities exist
		if(dim_ineqcon_conc_){
			//Create primal barrier derivative -tau*ci/dcidu
			double tmp_ci[dim_ineqcon_conc_];
			//Get constraint functions
			ci(tmp_ci,t,x,u,p,xdes,udes);

			//Get constraint functions
			for(int i=0;i<dim_ineqcon_conc_;i++){
				if(workingset_[i]){
					tmp_ci[i]=0;
				}
			}
			//Get derivative of inequality constraint with calculated factors
			dcidxmui	(tmp_x,t,x,u,p,xdes,udes,tmp_ci);
			for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]+=tmp_x[it_state];}
		}
		//Negate dHdx
		for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]=-out[it_state];}
	};





	/***************************************************************************************
	 *	Hamiltonian=l+lambda*f+mui*ci+mu*(c+slack^2)-k*a
	 *	=>dHdu=[dldu+lambda*dfdu+mui*dcidu+mu*dcdu;ci;cia;dciada
	 *	0=dHdu for optimality
	 ***************************************************************************************/
	inline void dHduExteriorPenalty(double  *out,double t, double *x, double *u, double *d, double *p, double *xdes, double *udes, double* lambda, double* mu, double* mui){
		double tmp_u[dim_u_conc_];
		memset(tmp_u, 0, dim_u_conc_*sizeof (double));
		//		for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;}

		//Get derivative of stage cost dldu
		dldu		(out,t,x,u,p,xdes,udes);
		//Get derivative of system function dfdx*lambda
		dfdulambda	(tmp_u,t,x,u,d,p,lambda);
		for(int it_control=0; it_control<dim_u_conc_; it_control++){out[it_control]+=tmp_u[it_control];}
		//Check if equalities exist
		if(dim_eqcon_conc_){
			//Get derivative of equality constraint dcidu
			dcdumu		(tmp_u,t,x,u,p,xdes,udes,mu);
			for(int it_control=0; it_control<dim_u_conc_; it_control++){out[it_control]+=tmp_u[it_control];}
			c(out+dim_u_conc_,t,x,u,p,xdes,udes);
		}
		//Check if inequalities exist
		if(dim_ineqcon_conc_){
			//Create primal barrier derivative -tau*ci/dcidu
			double tmp_ci[dim_ineqcon_conc_];
			//Get constraint functions
			ci(tmp_ci,t,x,u,p,xdes,udes);
			for(int it_constraint=0; it_constraint<dim_ineqcon_conc_; it_constraint++){
				if(tmp_ci[it_constraint]>0)
					tmp_ci[it_constraint]=exteriorpenaltyfactor_;
				else
					tmp_ci[it_constraint]=0;
			}
			//Get derivative of inequality constraint with calculated factors
			dcidumui	(tmp_u,t,x,u,p,xdes,udes,tmp_ci);
			for(int it_control=0; it_control<dim_u_conc_; it_control++){out[it_control]+=tmp_u[it_control];}
		}
	};
	/***************************************************************************************
	 *	Hamiltonian=l+lambda*f+mui*ci+mu*(c+slack^2)-k*a
	 *	=>minusdHdx=-[dldx+lambda*dfdx+mui*dcidx+mu*dcdx]
	 ***************************************************************************************/
	inline void minusdHdxExteriorPenalty(double  *out,double t, double *x, double *u, double *d, double *p, double *xdes, double *udes, double* lambda, double* mu, double* mui){
		double tmp_x[dim_x_conc_];
		memset(tmp_x, 0, dim_x_conc_*sizeof (double));
		//		for(int it_control=0; it_control<dim_x_conc_; it_control++){tmp_x[it_control]=0;}

		dldx		(out,t,x,u,p,xdes,udes);
		dfdxlambda	(tmp_x,t,x,u,d,p,lambda);
		for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]+=tmp_x[it_state];}
		if(dim_eqcon_conc_){
			dcdxmu		(tmp_x,t,x,u,p,xdes,udes,mu);
			for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]+=tmp_x[it_state];}
		}
		//Check if inequalities exist
		if(dim_ineqcon_conc_){
			//Create primal barrier derivative -tau*ci/dcidu
			double tmp_ci[dim_ineqcon_conc_];
			//Get constraint functions
			ci(tmp_ci,t,x,u,p,xdes,udes);
			for(int it_constraint=0; it_constraint<dim_ineqcon_conc_; it_constraint++){
				if(tmp_ci[it_constraint]>0){
					tmp_ci[it_constraint]=exteriorpenaltyfactor_;
				}
				else{
					tmp_ci[it_constraint]=0;
				}
			}
			//Get derivative of inequality constraint with calculated factors
			dcidxmui	(tmp_x,t,x,u,p,xdes,udes,tmp_ci);
			for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]+=tmp_x[it_state];}
		}
		//Negate dHdx
		for(int it_state=0; it_state<dim_x_conc_; it_state++){out[it_state]=-out[it_state];}
	};

	//	/***************************************************************************************
	//	 *	Hamiltonian=l+lambda*f+mui*ci+mu*c
	//	 *	=>dHdu=dldu+lambda*dfdu+mui*dcidu+mu*dcdu
	//	 *	0=dHdu for optimality
	//	 ***************************************************************************************/
	//	void dHdu(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes, double* lambda, double* mu, double* mui){
	//		double tmp_u[dim_u_conc_];
	//		//dldu
	//		dldu		(out,t,x,u,p,xdes,udes);
	//		//dfdu*lambda
	//		dfdulambda	(tmp_u,t,x,u,p,lambda);
	//		for(int it_control=0; it_control<dim_u_conc_; it_control++){
	//			out[it_control]+=tmp_u[it_control];
	//		}
	//		//dcdu*mu
	//		dcdumu		(tmp_u,t,x,u,p,xdes,udes,mu);
	//		for(int it_control=0; it_control<dim_u_conc_; it_control++){
	//			out[it_control]+=tmp_u[it_control];
	//		}
	//		//dcidu*mui
	//		dcidumui	(tmp_u,t,x,u,p,xdes,udes,mui);
	//		for(int it_control=0; it_control<dim_u_conc_; it_control++){
	//			out[it_control]+=tmp_u[it_control];
	//		}
	//	};
	//
	//	/***************************************************************************************
	//	 *	Hamiltonian=l+lambda*f+mui*ci+mu*c
	//	 *	=>dHdx=dldx+lambda*dfdx+mui*dcidx+mu*dcdx
	//	 *	dlambda=-dHdx for optimality
	//	 ***************************************************************************************/
	//	void dHdx(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes, double* lambda, double* mu, double* mui){
	//		double tmp_x[dim_x_conc_];
	//		dldx		(out,t,x,u,p,xdes,udes);
	//		dfdxlambda	(tmp_x,t,x,u,p,lambda);
	//		for(int it_state=0; it_state<dim_x_conc_; it_state++){
	//			out[it_state]+=tmp_x[it_state];
	//		}
	//		dcdxmu		(tmp_x,t,x,u,p,xdes,udes,mu);
	//		for(int it_state=0; it_state<dim_x_conc_; it_state++){
	//			out[it_state]+=tmp_x[it_state];
	//		}
	//		dcidxmui	(tmp_x,t,x,u,p,xdes,udes,mui);
	//		for(int it_state=0; it_state<dim_x_conc_; it_state++){
	//			out[it_state]+=tmp_x[it_state];
	//		}
	//	};


	/****** FUNCTIONS THAT ARE GENERATED WITH CODE GENERATION *****/
	//System function
	virtual void f(double *out, double t, double *x, double *u, double *d, double *p);
	virtual void dfdx(double **out, double t, double *x, double *u, double *d, double *p){std::cout<<"Controller::dfdx not defined!!!"<<std::endl;};
	virtual void dfdu(double **out, double t, double *x, double *u, double *d, double *p){std::cout<<"Controller::dfdu not defined!!!"<<std::endl;};
	virtual void dfdxlambda(double *out, double t, double *x, double *u, double *d, double *p, double *lambda);
	virtual void dfdulambda(double *out, double t, double *x, double *u, double *d, double *p, double *lambda);
	//Stage costs
	virtual void l(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes);
	virtual void dldx(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes);
	virtual void dldu(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes);
	//Final costs
	virtual void v(double *out, double t, double *x, double *p, double *xdes);
	virtual void dvdx(double *out, double t, double *x, double *p, double *xdes);
	//Equality constraints
	virtual void c(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes);
	virtual void dcdx(double **out, double t, double *x, double *u, double *p, double *xdes, double *udes){std::cout<<"Controller::dcdx not defined!!!"<<std::endl;};
	virtual void dcdu(double **out, double t, double *x, double *u, double *p, double *xdes, double *udes){std::cout<<"Controller::dcdu not defined!!!"<<std::endl;};
	virtual void dcdxmu(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes, double *mu);
	virtual void dcdumu(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes, double *mu);
	//Inequality constraints
	virtual void ci(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes);
	virtual void dcidx(double **out,double t, double *x, double *u, double *p, double *xdes, double *udes){std::cout<<"Controller::dcidx not defined!!!"<<std::endl;};
	virtual void dcidu(double **out,double t, double *x, double *u, double *p, double *xdes, double *udes){std::cout<<"Controller::dcidu not defined!!!"<<std::endl;};
	virtual void dcidxmui(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes, double *mui);
	virtual void dcidumui(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes, double *mui);
	//Inequality constraints with slacks
	virtual void cia(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes,double *slack);
	virtual void dciadx(double **out,double t, double *x, double *u, double *p, double *xdes, double *udes,double *slack){std::cout<<"Controller::dciadx not defined!!!"<<std::endl;};
	virtual void dciadu(double **out,double t, double *x, double *u, double *p, double *xdes, double *udes,double *slack){std::cout<<"Controller::dciadu not defined!!!"<<std::endl;};
	virtual void dciadxmui(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes,double *mu_ineq, double *slack);
	virtual void dciadumui(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes,double *mu_ineq, double *slack);
	virtual void dciadamui(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes,double *mu_ineq, double *slack);
};

#endif //_COMPOSER_H




