/**
 * @file    Controller.cpp
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

#include "Controller.h"

void Controller::startAgents(){
	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		tmp_agent_ptr_->start();
	}
}


	void Controller::addAgent(Agent* agent){
		agent->reset2initialstate();
		agentlist_.push_back(agent);
	};


void Controller::setActiveSetWorkingSet(double* mui){
	//Set working set
	for(int i=0;i<dim_ineqcon_conc_;i++){
		workingset_[i]=mui >= 0 ? true : false;
	}
}

void Controller::getMeasurements(){
	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		tmp_agent_ptr_->getDesiredState  (xdes_conc_+tmp_agent_ptr_->index_xdes_);
		tmp_agent_ptr_->getDesiredControl(udes_conc_+tmp_agent_ptr_->index_udes_);
		tmp_agent_ptr_->getState		 (x_conc_+tmp_agent_ptr_->index_x_);
//		tmp_agent_ptr_->getDisturbance	 (d_conc_+tmp_agent_ptr_->index_d_);
		tmp_agent_ptr_->getParameter(p_conc_+tmp_agent_ptr_->index_p_);
		//Loop over constraints
		for(int it_constraint=0; it_constraint<agentlist_[it_agent]->getConstraint_Dim(); it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			tmp_constraint_ptr_->getParameter(p_conc_+tmp_constraint_ptr_->index_pc_);
		}
		//Loop over couplings
		for(int it_coupling=0; it_coupling<tmp_agent_ptr_->getCoupling_Dim(); it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			if(tmp_agent_ptr_==tmp_coupling_ptr_->getAgent1()){
				tmp_coupling_ptr_->getParameter(p_conc_+tmp_coupling_ptr_->index_pc_);
			}
		}
	}
}

void Controller::applyAction(){
	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		tmp_agent_ptr_->setControl(u_conc_+tmp_agent_ptr_->index_u_);
		tmp_agent_ptr_->rosPublishActuation();
	}
};

Controller::~Controller(){
	//Typicall u,mu,mui,slack are concatenated in the optvar_conc_
	MathLib::free(optvar_conc_);
	MathLib::free(x_conc_);
	u_conc_   	=NULL;
	MathLib::free(xdes_conc_);
	MathLib::free(udes_conc_);
	MathLib::free(d_conc_);
	MathLib::free(p_conc_);
	MathLib::free(lambda_conc_);
	mu_conc_		=NULL;
	mui_conc_	=NULL;
	slack_conc_	=NULL;
	MathLib::free(slack_penalty_conc_);
}

Controller::Controller(std::vector<Agent*> _listofagents, int _id=0){
	id_=_id;
	agentlist_=_listofagents;
	primalbarrierfactor_=0.0001;//0.001;
	exteriorpenaltyfactor_=100;
	mu_init_		=0.0001;
	mui_init_ 	    =0.0001;
	slack_init_ 	=0.01;
	slack_penalty_init_=0.01;

	//log_stringstream_<<"Controller Log File:"<<std::endl<<std::endl;

	flag_show_controllerinfo_=false;//true;
	flag_show_controllerstates_=false;
	flag_save_controllerlog_=false;
	flag_show_controllertrace_=false;

	if(flag_show_controllertrace_){
		flag_show_controllerinfo_=true;
		flag_show_controllerstates_=true;
	}
//	if(flag_save_controllerlog_){

		time_t currenttimestamp;
		tm *now;
		currenttimestamp = time(0);
		now = localtime(&currenttimestamp);
		std::stringstream filename;
		filename<<now->tm_year+1900<<boost::format("%|02|")%(now->tm_mon+1)\
					<<boost::format("%|02|")%(now->tm_mday)<<"_"\
					<<boost::format("%|02|")%(now->tm_hour)\
					<<boost::format("%|02|")%(now->tm_min)\
					<<boost::format("%|02|")%(now->tm_sec)<<"_"\
					<<"controller"<<id_<<".log";
		log_filename_=filename.str();
//	}
	std::stringstream rostopic_computationtime;
	rostopic_computationtime<<"controller"<<id_<<"_computation_time";
	comp_time_publisher_=node_.advertise<std_msgs::Float32>(rostopic_computationtime.str(), 1);

	constrainthandlingmethod_=METHOD_PRIMALBARRIER;
//	constrainthandlingmethod_=METHOD_EXTERIORPENALTY;
//	constrainthandlingmethod_=METHOD_AUXILIARYVARIABLE;

	//Set Agent states to initial values
	for(int it_agent=0; it_agent<agentlist_.size(); it_agent++){
		agentlist_[it_agent]->reset2initialstate();
	}
	//Initialize Concatenation Indices
	initConcatenation();
	//	//Save Agent Arrays in concatenated arrays
	//	initArrayValues();
}

void Controller::initConcatenation(){

	dim_x_conc_		=0;
	dim_u_conc_		=0;
	dim_xdes_conc_	=0;
	dim_udes_conc_	=0;
	dim_d_conc_		=0;
	dim_p_conc_		=0;
	dim_lambda_conc_=0;
	dim_eqcon_conc_	=0;
	dim_ineqcon_conc_=0;
	//Loop through agents
	for(int it_agent=0; it_agent<agentlist_.size(); it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		//Get index
		tmp_agent_ptr_->initIndex(dim_x_conc_,dim_u_conc_,dim_xdes_conc_,dim_udes_conc_,dim_d_conc_,dim_p_conc_,dim_lambda_conc_);
		dim_x_conc_   	 +=agentlist_[it_agent]->getState_Dim();
		dim_u_conc_   	 +=agentlist_[it_agent]->getControl_Dim();
		dim_xdes_conc_	 +=agentlist_[it_agent]->getDesiredState_Dim();
		dim_udes_conc_	 +=agentlist_[it_agent]->getDesiredControl_Dim();
		dim_d_conc_   	 +=agentlist_[it_agent]->getDisturbance_Dim();
		dim_p_conc_   	 +=agentlist_[it_agent]->getParameter_Dim();
		dim_lambda_conc_ +=agentlist_[it_agent]->getState_Dim();
	}

	//Loop through constraints
	for(int it_agent=0; it_agent<agentlist_.size(); it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		//Loop through constraints
		for(int it_constraint=0; it_constraint<agentlist_[it_agent]->getConstraint_Dim(); it_constraint++){
			//Get index
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			tmp_constraint_ptr_->initIndex(
					tmp_agent_ptr_->index_x_,
					tmp_agent_ptr_->index_u_,
					tmp_agent_ptr_->index_xdes_,
					tmp_agent_ptr_->index_udes_,
					tmp_agent_ptr_->index_p_,
					dim_p_conc_,
					dim_eqcon_conc_,
					dim_ineqcon_conc_);
			//Concatenate equality lagrangemultipliers
			dim_eqcon_conc_ +=agentlist_[it_agent]->constraint_[it_constraint]->getEqualityConstraint_Dim();
			//Concatenate inequality lagrangemultipliers
			dim_ineqcon_conc_ +=agentlist_[it_agent]->constraint_[it_constraint]->getInequalityConstraint_Dim();
			//Concatenate parameters
			dim_p_conc_+=tmp_constraint_ptr_->getParameter_Dim();
		}
	}
	//Loop through couplings
	// Has to be done after agent initialisation, because it refers to the agent indices
	for(int it_agent=0; it_agent<agentlist_.size(); it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		//Loop over couplings of agent
		for(int it_coupling=0; it_coupling<tmp_agent_ptr_->getCoupling_Dim(); it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			if(tmp_agent_ptr_==tmp_coupling_ptr_->getAgent1()){
				Agent* agent1=tmp_coupling_ptr_->getAgent1();
				Agent* agent2=tmp_coupling_ptr_->getAgent2();
				tmp_coupling_ptr_->initIndex(
						agent1->index_x_,agent2->index_x_,
						agent1->index_u_,agent2->index_u_,
						agent1->index_xdes_,agent2->index_xdes_,
						agent1->index_udes_,agent2->index_udes_,
						agent1->index_p_,agent2->index_p_,
						dim_p_conc_,dim_eqcon_conc_,dim_ineqcon_conc_);
				//Concatenate equality lagrangemultipliers
				dim_eqcon_conc_ +=tmp_coupling_ptr_->getEqualityConstraint_Dim();
				//Concatenate inequality lagrangemultipliers
				dim_ineqcon_conc_ +=tmp_coupling_ptr_->getInequalityConstraint_Dim();
				//Concatenate parameters
				dim_p_conc_+=tmp_coupling_ptr_->getParameter_Dim();
			}
		}
	}

	switch(constrainthandlingmethod_){
	case METHOD_PRIMALBARRIER     :
		//The optimized variables are inputs and equality constraint multipliers
		dim_optvar_conc_=dim_u_conc_+dim_eqcon_conc_;
		optvar_conc_	=MathLib::defvector(dim_optvar_conc_);
		x_conc_   	=MathLib::defvector(dim_x_conc_);
		u_conc_   	=optvar_conc_;
		d_conc_	  	=MathLib::defvector(dim_d_conc_);
		p_conc_	  	=MathLib::defvector(dim_p_conc_);
		xdes_conc_	=MathLib::defvector(dim_xdes_conc_);
		udes_conc_	=MathLib::defvector(dim_udes_conc_);
		lambda_conc_	=MathLib::defvector(dim_lambda_conc_);
		mu_conc_		=optvar_conc_+dim_u_conc_;
		//INITIALIZE
		//Loop through agents
		for(int it_agent=0; it_agent<agentlist_.size(); it_agent++){
			tmp_agent_ptr_=agentlist_[it_agent];
			//Get values from agent and store them in concatenated arrays
			tmp_agent_ptr_->getState(x_conc_+tmp_agent_ptr_->index_x_);
			tmp_agent_ptr_->getControl(u_conc_+tmp_agent_ptr_->index_u_);
			tmp_agent_ptr_->getDesiredState(xdes_conc_+tmp_agent_ptr_->index_xdes_);
			tmp_agent_ptr_->getDesiredControl(udes_conc_+tmp_agent_ptr_->index_udes_);
			tmp_agent_ptr_->getParameter(p_conc_+tmp_agent_ptr_->index_p_);
			//Loop over constraints
			for(int it_constraint=0; it_constraint<agentlist_[it_agent]->getConstraint_Dim(); it_constraint++){
				tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
				tmp_constraint_ptr_->getParameter(p_conc_+tmp_constraint_ptr_->index_pc_);
			}
			//Loop over couplings
			for(int it_coupling=0; it_coupling<tmp_agent_ptr_->getCoupling_Dim(); it_coupling++){
				tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
				if(tmp_agent_ptr_==tmp_coupling_ptr_->getAgent1()){
					tmp_coupling_ptr_->getParameter(p_conc_+tmp_coupling_ptr_->index_pc_);
				}
			}
		}
		//Initialize mu
		for(int it_eq=0;it_eq<dim_eqcon_conc_ ;it_eq++){
			mu_conc_[it_eq]=mu_init_;
		}

		break;

	case METHOD_AUXILIARYVARIABLE :
		//The optimized variables are inputs and equality constraint multipliers, inequality constraint multipliers and slack variables
		dim_optvar_conc_=dim_u_conc_+dim_eqcon_conc_+2*dim_ineqcon_conc_;
		optvar_conc_	=MathLib::defvector(dim_optvar_conc_);
		x_conc_   	=MathLib::defvector(dim_x_conc_);
		u_conc_   	=optvar_conc_;
		xdes_conc_	=MathLib::defvector(dim_xdes_conc_);
		udes_conc_	=MathLib::defvector(dim_udes_conc_);
		d_conc_	  	=MathLib::defvector(dim_d_conc_);
		p_conc_	  	=MathLib::defvector(dim_p_conc_);
		lambda_conc_	=MathLib::defvector(dim_lambda_conc_);
		mu_conc_		=optvar_conc_+dim_u_conc_;
		mui_conc_	=optvar_conc_+dim_u_conc_+dim_eqcon_conc_;
		slack_conc_	=optvar_conc_+dim_u_conc_+dim_eqcon_conc_+dim_ineqcon_conc_;
		slack_penalty_conc_=MathLib::defvector(dim_ineqcon_conc_);
		//INITIALIZE
		//Loop through agents
		for(int it_agent=0; it_agent<agentlist_.size(); it_agent++){
			tmp_agent_ptr_=agentlist_[it_agent];
			//Get values from agent and store them in concatenated arrays
			tmp_agent_ptr_->getState(x_conc_+tmp_agent_ptr_->index_x_);
			tmp_agent_ptr_->getControl(u_conc_+tmp_agent_ptr_->index_u_);
			tmp_agent_ptr_->getDesiredState(xdes_conc_+tmp_agent_ptr_->index_xdes_);
			tmp_agent_ptr_->getDesiredControl(udes_conc_+tmp_agent_ptr_->index_udes_);
			tmp_agent_ptr_->getParameter(p_conc_+tmp_agent_ptr_->index_p_);
			//Loop over constraints
			for(int it_constraint=0; it_constraint<agentlist_[it_agent]->getConstraint_Dim(); it_constraint++){
				tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
				tmp_constraint_ptr_->getParameter(p_conc_+tmp_constraint_ptr_->index_pc_);
			}
			//Loop over couplings
			for(int it_coupling=0; it_coupling<tmp_agent_ptr_->getCoupling_Dim(); it_coupling++){
				tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
				if(tmp_agent_ptr_==tmp_coupling_ptr_->getAgent1()){
					tmp_coupling_ptr_->getParameter(p_conc_+tmp_coupling_ptr_->index_pc_);
				}
			}
		}
		//Initialize mu
		for(int it_eq=0;it_eq<dim_eqcon_conc_ ;it_eq++){
			mu_conc_[it_eq]=mu_init_;
		}
		//Initialize mui
		for(int it_ineq=0;it_ineq<dim_ineqcon_conc_ ;it_ineq++){
			mui_conc_[it_ineq]=mui_init_;
		}
		//Initialize slack
		for(int it_ineq=0;it_ineq<dim_ineqcon_conc_ ;it_ineq++){
			slack_conc_[it_ineq]=slack_init_;
			slack_penalty_conc_[it_ineq]=slack_penalty_init_;
		}
		break;

	case METHOD_ACTIVESET         :
		//The optimized variables are inputs and equality constraint multipliers, inequality constraint multipliers
		dim_optvar_conc_=dim_u_conc_+dim_eqcon_conc_+dim_ineqcon_conc_;
		optvar_conc_=MathLib::defvector(dim_optvar_conc_);
		x_conc_   	=MathLib::defvector(dim_x_conc_);
		u_conc_   	=optvar_conc_;
		xdes_conc_	=MathLib::defvector(dim_xdes_conc_);
		udes_conc_	=MathLib::defvector(dim_udes_conc_);
		p_conc_	  	=MathLib::defvector(dim_p_conc_);
		d_conc_	  	=MathLib::defvector(dim_d_conc_);
		lambda_conc_=MathLib::defvector(dim_lambda_conc_);
		mu_conc_	=optvar_conc_+dim_u_conc_;
		mui_conc_	=optvar_conc_+dim_u_conc_+dim_eqcon_conc_;
		workingset_ =MathLib::defboolvector(dim_ineqcon_conc_);

		//INITIALIZE
		//Loop through agents
		for(int it_agent=0; it_agent<agentlist_.size(); it_agent++){
			tmp_agent_ptr_=agentlist_[it_agent];
			//Get values from agent and store them in concatenated arrays
			tmp_agent_ptr_->getState(x_conc_+tmp_agent_ptr_->index_x_);
			tmp_agent_ptr_->getControl(u_conc_+tmp_agent_ptr_->index_u_);
			tmp_agent_ptr_->getDesiredState(xdes_conc_+tmp_agent_ptr_->index_xdes_);
			tmp_agent_ptr_->getDesiredControl(udes_conc_+tmp_agent_ptr_->index_udes_);
			tmp_agent_ptr_->getParameter(p_conc_+tmp_agent_ptr_->index_p_);
			//Loop over constraints
			for(int it_constraint=0; it_constraint<agentlist_[it_agent]->getConstraint_Dim(); it_constraint++){
				tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
				tmp_constraint_ptr_->getParameter(p_conc_+tmp_constraint_ptr_->index_pc_);
			}
			//Loop over couplings
			for(int it_coupling=0; it_coupling<tmp_agent_ptr_->getCoupling_Dim(); it_coupling++){
				tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
				if(tmp_agent_ptr_==tmp_coupling_ptr_->getAgent1()){
					tmp_coupling_ptr_->getParameter(p_conc_+tmp_coupling_ptr_->index_pc_);
				}
			}
		}
		//Initialize mu
		for(int it_eq=0;it_eq<dim_eqcon_conc_ ;it_eq++){
			mu_conc_[it_eq]=mu_init_;
		}
		//Initialize mui
		for(int it_ineq=0;it_ineq<dim_ineqcon_conc_ ;it_ineq++){
			mui_conc_[it_ineq]=mui_init_;
		}

		break;

	case METHOD_EXTERIORPENALTY         :
		//The optimized variables are inputs and equality constraint multipliers
		dim_optvar_conc_=dim_u_conc_+dim_eqcon_conc_;
		optvar_conc_	=MathLib::defvector(dim_optvar_conc_);
		x_conc_   		=MathLib::defvector(dim_x_conc_);
		u_conc_   		=optvar_conc_;
		d_conc_	  		=MathLib::defvector(dim_d_conc_);
		p_conc_	  		=MathLib::defvector(dim_p_conc_);
		xdes_conc_		=MathLib::defvector(dim_xdes_conc_);
		udes_conc_		=MathLib::defvector(dim_udes_conc_);
		lambda_conc_	=MathLib::defvector(dim_lambda_conc_);
		mu_conc_		=optvar_conc_+dim_u_conc_;
		//INITIALIZE
		//Loop through agents
		for(int it_agent=0; it_agent<agentlist_.size(); it_agent++){
			tmp_agent_ptr_=agentlist_[it_agent];
			//Get values from agent and store them in concatenated arrays
			tmp_agent_ptr_->getState(x_conc_+tmp_agent_ptr_->index_x_);
			tmp_agent_ptr_->getControl(u_conc_+tmp_agent_ptr_->index_u_);
			tmp_agent_ptr_->getDesiredState(xdes_conc_+tmp_agent_ptr_->index_xdes_);
			tmp_agent_ptr_->getDesiredControl(udes_conc_+tmp_agent_ptr_->index_udes_);
			tmp_agent_ptr_->getParameter(p_conc_+tmp_agent_ptr_->index_p_);
			//Loop over constraints
			for(int it_constraint=0; it_constraint<agentlist_[it_agent]->getConstraint_Dim(); it_constraint++){
				tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
				tmp_constraint_ptr_->getParameter(p_conc_+tmp_constraint_ptr_->index_pc_);
			}
			//Loop over couplings
			for(int it_coupling=0; it_coupling<tmp_agent_ptr_->getCoupling_Dim(); it_coupling++){
				tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
				if(tmp_agent_ptr_==tmp_coupling_ptr_->getAgent1()){
					tmp_coupling_ptr_->getParameter(p_conc_+tmp_coupling_ptr_->index_pc_);
				}
			}
		}
		//Initialize mu
		for(int it_eq=0;it_eq<dim_eqcon_conc_ ;it_eq++){
			mu_conc_[it_eq]=mu_init_;
		}

		break;

	default:
		std::cout<<"!!! ERROR: COMPOSER::init() NO VALID INEQUALITY CONSTRAINT METHOD CHOSEN"<<std::endl;
	}
}


/***** system function f *****/
void Controller::f			(double *out,double t,double *x, double *u, double *d, double *p){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::f()"<<std::endl;
#endif
	for(int i=0;i<agentlist_.size();i++){
		tmp_agent_ptr_=agentlist_[i];
		agentlist_[i]->f(out+tmp_agent_ptr_->index_x_,t,x+tmp_agent_ptr_->index_x_,u+tmp_agent_ptr_->index_u_,d+tmp_agent_ptr_->index_d_,p+tmp_agent_ptr_->index_p_);
	}
}
void Controller::dfdxlambda	(double *out,double t,double *x, double *u, double *d, double *p, double *lambda){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::dfdx_lambda()"<<std::endl;
#endif
	double tmp_x[dim_x_conc_];
	//Loop over agents
	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		//Check if state derivative is non-zero
		if(tmp_agent_ptr_->dim_x_>0){
			//Get function values
			tmp_agent_ptr_->dfdxlambda(
					tmp_x+tmp_agent_ptr_->index_x_,
					t,
					x+tmp_agent_ptr_->index_x_,
					u+tmp_agent_ptr_->index_u_,
					d+tmp_agent_ptr_->index_d_,
					p+tmp_agent_ptr_->index_p_,
					lambda+tmp_agent_ptr_->index_lambda_);
			//Save tmp to output and reset tmp
			for(int it_state=tmp_agent_ptr_->index_x_; it_state<tmp_agent_ptr_->index_x_+tmp_agent_ptr_->dim_x_; it_state++){
				out[it_state]+=tmp_x[it_state];
			}
		}
	}
}
void Controller::dfdulambda	(double *out,double t,double *x, double *u, double *d, double *p, double *lambda){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::dfdu_lambda()"<<std::endl;
#endif
	double tmp_u[dim_u_conc_];
	//Loop over agents
	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		//Check if state derivative is non-zero
		if(tmp_agent_ptr_->dim_u_>0){
			//Get function values
			tmp_agent_ptr_->dfdulambda(
					tmp_u+tmp_agent_ptr_->index_u_,
					t,
					x+tmp_agent_ptr_->index_x_,
					u+tmp_agent_ptr_->index_u_,
					d+tmp_agent_ptr_->index_d_,
					p+tmp_agent_ptr_->index_p_,
					lambda+tmp_agent_ptr_->index_lambda_);
			//Save tmp to output
			for(int it_control=tmp_agent_ptr_->index_u_; it_control<tmp_agent_ptr_->index_u_+tmp_agent_ptr_->dim_u_; it_control++){
				out[it_control]+=tmp_u[it_control];
			}
		}
	}
}
/***** state cost l *****/
void Controller::l	(double *out,double t,double *x, double *u, double *p, double *xdes, double *udes){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::l()"<<std::endl;
#endif
	double tmp=0;
	*out=0;
	//Loop over agents
	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		//if(tmp_agent_ptr_->dim_l>0){
		tmp_agent_ptr_->l(&tmp,t,x+tmp_agent_ptr_->index_x_,u+tmp_agent_ptr_->index_u_,p+tmp_agent_ptr_->index_p_,xdes+tmp_agent_ptr_->index_xdes_,udes+tmp_agent_ptr_->index_udes_);
		*out+=tmp;
		tmp=0;
		//}
		//Loop over constraints in agents
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->dim_l_>0){
				tmp_constraint_ptr_->l(&tmp,t,x+tmp_constraint_ptr_->index_x_,u+tmp_constraint_ptr_->index_u_,p+tmp_constraint_ptr_->index_p_,p+tmp_constraint_ptr_->index_pc_,xdes+tmp_constraint_ptr_->index_xdes_,udes+tmp_constraint_ptr_->index_udes_);
				*out+=tmp;
				tmp=0;
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_l_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)){
				tmp_coupling_ptr_->l(&tmp,t,
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,
						u+tmp_coupling_ptr_->index_u1_,x+tmp_coupling_ptr_->index_u2_,
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_,
						udes+tmp_coupling_ptr_->index_udes1_,udes+tmp_coupling_ptr_->index_udes2_);
				*out+=tmp;
				tmp=0;
			}
		}
	}
}
void Controller::dldx	(double *out,double t,double *x, double *u, double *p, double *xdes, double *udes){
	// printf("// -- -- -- -- void Controller::dldx\n");
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::dldx()"<<std::endl;
#endif
	double tmp_x[dim_x_conc_];
	memset(out,   0, dim_x_conc_*sizeof (double));
	memset(tmp_x, 0, dim_x_conc_*sizeof (double));
//	for(int it_state=0; it_state<dim_x_conc_; it_state++){tmp_x[it_state]=0;out[it_state]=0;};

	//Loop over agents
	// printf("// -- -- -- -- -- Loop over agents\n");
	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		if(tmp_agent_ptr_->dim_x_>0){
			//Get function values
			tmp_agent_ptr_->dldx(
					tmp_x+tmp_agent_ptr_->index_x_,
					t,
					x+tmp_agent_ptr_->index_x_,
					u+tmp_agent_ptr_->index_u_,
					p+tmp_agent_ptr_->index_p_,
					xdes+tmp_agent_ptr_->index_xdes_,
					udes+tmp_agent_ptr_->index_udes_);
			//Save tmp to output and reset tmp
			for(int it_state=tmp_agent_ptr_->index_x_; it_state<tmp_agent_ptr_->index_x_+tmp_agent_ptr_->dim_x_; it_state++){
				out[it_state]+=tmp_x[it_state];
			}
		}
		//Loop over constraints in agents
		// printf("// -- -- -- -- -- Loop over constraints in agents\n");
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			// printf("// -- -- -- -- -- -- tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];\n");
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			// printf("// -- -- -- -- -- -- if(tmp_constraint_ptr_->dim_l_>0&&tmp_constraint_ptr_->dim_x_>0)\n");
			if(tmp_constraint_ptr_->dim_l_>0&&tmp_constraint_ptr_->dim_x_>0){
				//Get function values
				// printf("// -- -- -- -- -- -- tmp_constraint_ptr_->dldx(\n");
				tmp_constraint_ptr_->dldx(
						tmp_x+tmp_constraint_ptr_->index_x_,
						t,
						x+tmp_constraint_ptr_->index_x_,
						u+tmp_constraint_ptr_->index_u_,
						p+tmp_constraint_ptr_->index_p_,
						p+tmp_constraint_ptr_->index_pc_,
						xdes+tmp_constraint_ptr_->index_xdes_,
						udes+tmp_constraint_ptr_->index_udes_);
				//Save tmp to output and reset tmp
				// printf("// -- -- -- -- -- -- Save tmp to output and reset tmp\n");
				for(int it_state=tmp_constraint_ptr_->index_x_; it_state<tmp_constraint_ptr_->index_x_+tmp_agent_ptr_->dim_x_; it_state++){
					out[it_state]+=tmp_x[it_state];
				}
			}
		}
		//Loop over couplings in agents
		// printf("// -- -- -- -- -- Loop over couplings in agents\n");
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_l_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)
					&&((tmp_agent_ptr_->dim_x_>0)||(tmp_coupling_ptr_->agent2_->dim_x_>0))){
				double tmp_dldx1[tmp_agent_ptr_->dim_x_];
				double tmp_dldx2[tmp_coupling_ptr_->agent2_->dim_x_];
				tmp_coupling_ptr_->dldx(tmp_dldx1,tmp_dldx2,t,\
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,\
						u+tmp_coupling_ptr_->index_u1_,u+tmp_coupling_ptr_->index_u2_,\
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,\
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_,\
						udes+tmp_coupling_ptr_->index_udes1_,udes+tmp_coupling_ptr_->index_udes2_);
				//Save tmp to output and reset tmp
				for(int it_state=0; it_state<tmp_agent_ptr_->dim_x_; it_state++){
					out[tmp_coupling_ptr_->index_x1_+it_state]+=tmp_dldx1[it_state];
				}
				for(int it_state=0; it_state<tmp_coupling_ptr_->agent2_->dim_x_; it_state++){
					out[tmp_coupling_ptr_->index_x2_+it_state]+=tmp_dldx2[it_state];
				}
			}
		}
	}
}
void Controller::dldu	(double *out,double t,double *x, double *u, double *p, double *xdes, double *udes){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::dldu()"<<std::endl;
#endif
	double tmp_u[dim_u_conc_];
	memset(out,   0, dim_u_conc_*sizeof (double));
	memset(tmp_u, 0, dim_u_conc_*sizeof (double));
//	for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;out[it_control]=0;};

	//Loop over agents
	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		if(tmp_agent_ptr_->dim_u_>0){
			//Get function values
			tmp_agent_ptr_->dldu(
					tmp_u+tmp_agent_ptr_->index_u_,
					t,
					x+tmp_agent_ptr_->index_x_,
					u+tmp_agent_ptr_->index_u_,
					p+tmp_agent_ptr_->index_p_,
					xdes+tmp_agent_ptr_->index_xdes_,
					udes+tmp_agent_ptr_->index_udes_);
			//Save tmp to output and reset tmp
			for(int it_control=tmp_agent_ptr_->index_u_; it_control<tmp_agent_ptr_->index_u_+tmp_agent_ptr_->dim_u_; it_control++){
				out[it_control]+=tmp_u[it_control];
			}
		}
		//Loop over constraints in agents
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->dim_l_>0&&tmp_constraint_ptr_->dim_u_>0){
				//Get function values
				tmp_agent_ptr_->constraint_[it_constraint]->dldu(
						tmp_u+tmp_constraint_ptr_->index_u_,
						t,
						x+tmp_constraint_ptr_->index_x_,
						u+tmp_constraint_ptr_->index_u_,
						p+tmp_constraint_ptr_->index_p_,
						p+tmp_constraint_ptr_->index_pc_,
						xdes+tmp_constraint_ptr_->index_xdes_,
						udes+tmp_constraint_ptr_->index_udes_);
				//Add tmp to out and reset tmp
				for(int it_control=tmp_constraint_ptr_->index_u_; it_control<tmp_constraint_ptr_->index_u_+tmp_agent_ptr_->dim_u_; it_control++){
					out[it_control]+=tmp_u[it_control];
				}
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_l_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)
					&&((tmp_agent_ptr_->dim_u_>0)||(tmp_coupling_ptr_->agent2_->dim_u_>0))){
				double tmp_dldu1[tmp_agent_ptr_->dim_u_];
				double tmp_dldu2[tmp_coupling_ptr_->agent2_->dim_u_];
				tmp_coupling_ptr_->dldx(tmp_dldu1,tmp_dldu2,t,\
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,\
						u+tmp_coupling_ptr_->index_u1_,u+tmp_coupling_ptr_->index_u2_,\
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,\
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_,\
						udes+tmp_coupling_ptr_->index_udes1_,udes+tmp_coupling_ptr_->index_udes2_);
				//Save tmp to output and reset tmp
				for(int it_control=0; it_control<tmp_agent_ptr_->dim_u_; it_control++){
					out[tmp_coupling_ptr_->index_u1_+it_control]+=tmp_dldu1[it_control];
				}
				for(int it_control=0; it_control<tmp_coupling_ptr_->agent2_->dim_u_; it_control++){
					out[tmp_coupling_ptr_->index_u2_+it_control]+=tmp_dldu2[it_control];
				}
			}
		}
	}
}
/***** final cost v *****/
void Controller::v	(double *out,double t,double *x, double *p, double *xdes){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::v()"<<std::endl;
#endif
	double tmp=0;
	*out=0;
	//Loop over agents
	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		tmp_agent_ptr_->v(&tmp,t,x+tmp_agent_ptr_->index_x_,p+tmp_agent_ptr_->index_p_,xdes+tmp_agent_ptr_->index_xdes_);
		*out+=tmp;
		tmp=0;
		//Loop over constraints in agents
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->dim_v_>0){
				tmp_constraint_ptr_->v(&tmp,t,x+tmp_constraint_ptr_->index_x_,p+tmp_constraint_ptr_->index_p_,p+tmp_constraint_ptr_->index_pc_,xdes+tmp_constraint_ptr_->index_xdes_);
				*out+=tmp;
				tmp=0;
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_v_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)){
				tmp_coupling_ptr_->v(&tmp,t,
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_);
				*out+=tmp;
				tmp=0;
			}
		}
	}
}
void Controller::dvdx	(double *out,double t,double *x,double *p, double *xdes){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::dvdx()"<<std::endl;
#endif
	double tmp_x[dim_x_conc_];
	memset(out,   0, dim_x_conc_*sizeof (double));
	memset(tmp_x, 0, dim_x_conc_*sizeof (double));
//	for(int it_state=0; it_state<dim_x_conc_; it_state++){tmp_x[it_state]=0;out[it_state]=0;};

	//Loop over agents
	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		if(tmp_agent_ptr_->dim_x_>0){
			//Get function values
			tmp_agent_ptr_->dvdx(
					tmp_x+tmp_agent_ptr_->index_x_,
					t,
					x+tmp_agent_ptr_->index_x_,
					p+tmp_agent_ptr_->index_p_,
					xdes+tmp_agent_ptr_->index_xdes_);
			//Save tmp to output
			for(int it_state=tmp_agent_ptr_->index_x_; it_state<tmp_agent_ptr_->index_x_+tmp_agent_ptr_->dim_x_; it_state++){
				out[it_state]+=tmp_x[it_state];
			}
		}
		//Loop over constraints in agents
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->getFinalCost_Dim()>0 &&tmp_constraint_ptr_->dim_x_>0 ){
				//Get function values
				tmp_constraint_ptr_->dvdx(
						tmp_x+tmp_constraint_ptr_->index_x_,
						t,
						x+tmp_constraint_ptr_->index_x_,
						p+tmp_constraint_ptr_->index_p_,
						p+tmp_constraint_ptr_->index_pc_,
						xdes+tmp_constraint_ptr_->index_xdes_);
				//Add tmp to out and reset tmp
				for(int it_state=tmp_constraint_ptr_->index_x_; it_state<tmp_constraint_ptr_->index_x_+agentlist_[it_agent]->dim_x_; it_state++){
					out[it_state]+=tmp_x[it_state];
				}
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_v_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)
					&&((tmp_agent_ptr_->dim_x_>0)||(tmp_coupling_ptr_->agent2_->dim_x_>0))){
				double tmp_dvdx1[tmp_agent_ptr_->dim_x_];
				double tmp_dvdx2[tmp_coupling_ptr_->agent2_->dim_x_];
				tmp_coupling_ptr_->dvdx(tmp_dvdx1,tmp_dvdx2,t,\
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,\
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,\
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_);
				//Save tmp to output and reset tmp
				for(int it_state=0; it_state<tmp_agent_ptr_->dim_x_; it_state++){
					out[tmp_coupling_ptr_->index_x1_+it_state]+=tmp_dvdx1[it_state];
				}
				for(int it_state=0; it_state<tmp_coupling_ptr_->agent2_->dim_x_; it_state++){
					out[tmp_coupling_ptr_->index_x2_+it_state]+=tmp_dvdx2[it_state];
				}
			}
		}
	}
}
/***** equality constraint *****/
void Controller::c		(double *out,double t,double *x, double *u, double *p, double *xdes, double *udes){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::c()"<<std::endl;
#endif
	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->dim_eq_>0){
				//Call function
				tmp_constraint_ptr_->c(
						out+tmp_constraint_ptr_->index_mu_,
						t,
						x+tmp_constraint_ptr_->index_x_,
						u+tmp_constraint_ptr_->index_u_,
						p+tmp_constraint_ptr_->index_p_,
						p+tmp_constraint_ptr_->index_pc_,
						xdes+tmp_constraint_ptr_->index_xdes_,
						udes+tmp_constraint_ptr_->index_udes_);
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_eq_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)){
				tmp_coupling_ptr_->c(out+tmp_coupling_ptr_->index_mu_,t,
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,
						u+tmp_coupling_ptr_->index_u1_,u+tmp_coupling_ptr_->index_u2_,
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_,
						udes+tmp_coupling_ptr_->index_udes1_,udes+tmp_coupling_ptr_->index_udes2_);
			}
		}
	}
}
void Controller::dcdxmu	(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes, double *mu){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::dcdx_mu()"<<std::endl;
#endif
	double tmp_x[dim_x_conc_];
	memset(out,   0, dim_x_conc_*sizeof (double));
	memset(tmp_x, 0, dim_x_conc_*sizeof (double));
//	for(int it_state=0; it_state<dim_x_conc_; it_state++){tmp_x[it_state]=0;out[it_state]=0;};

	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->dim_eq_>0&&tmp_constraint_ptr_->dim_x_>0){
				//Call function
				tmp_constraint_ptr_->dcdxmu(
						tmp_x+tmp_constraint_ptr_->index_x_,
						t,
						x+tmp_constraint_ptr_->index_x_,
						u+tmp_constraint_ptr_->index_u_,
						p+tmp_constraint_ptr_->index_p_,
						p+tmp_constraint_ptr_->index_pc_,
						xdes+tmp_constraint_ptr_->index_xdes_,
						udes+tmp_constraint_ptr_->index_udes_,
						mu+tmp_constraint_ptr_->index_mu_);
				//Add tmp to out and reset tmp
				for(int it_state=tmp_constraint_ptr_->index_x_; it_state<tmp_constraint_ptr_->index_x_+agentlist_[it_agent]->dim_x_; it_state++){
					out[it_state]+=tmp_x[it_state];
				}
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_eq_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)
					&&((tmp_agent_ptr_->dim_x_>0)||(tmp_coupling_ptr_->agent2_->dim_x_>0))){
				double tmp_dcdx1mu[tmp_agent_ptr_->dim_x_];
				double tmp_dcdx2mu[tmp_coupling_ptr_->agent2_->dim_x_];
				tmp_coupling_ptr_->dcdxmu(tmp_dcdx1mu,tmp_dcdx2mu,t,
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,
						u+tmp_coupling_ptr_->index_u1_,u+tmp_coupling_ptr_->index_u2_,
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_,
						udes+tmp_coupling_ptr_->index_udes1_,udes+tmp_coupling_ptr_->index_udes2_,
						mu+tmp_coupling_ptr_->index_mu_);
				//Save tmp to output and reset tmp
				for(int it_state=0; it_state<tmp_agent_ptr_->dim_x_; it_state++){
					out[tmp_coupling_ptr_->index_x1_+it_state]+=tmp_dcdx1mu[it_state];
				}
				for(int it_state=0; it_state<tmp_coupling_ptr_->agent2_->dim_x_; it_state++){
					out[tmp_coupling_ptr_->index_x2_+it_state]+=tmp_dcdx2mu[it_state];
				}
			}
		}
	}
}
void Controller::dcdumu	(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes, double *mu){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::dcdu_mu()"<<std::endl;
#endif
	double tmp_u[dim_u_conc_];
	memset(out,   0, dim_u_conc_*sizeof (double));
	memset(tmp_u, 0, dim_u_conc_*sizeof (double));
//	for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;out[it_control]=0;};

	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->dim_eq_>0&&tmp_constraint_ptr_->dim_u_>0){
				//Call function
				tmp_constraint_ptr_->dcdumu(
						tmp_u+tmp_constraint_ptr_->index_u_,
						t,
						x+tmp_constraint_ptr_->index_x_,
						u+tmp_constraint_ptr_->index_u_,
						p+tmp_constraint_ptr_->index_p_,
						p+tmp_constraint_ptr_->index_pc_,
						xdes+tmp_constraint_ptr_->index_xdes_,
						udes+tmp_constraint_ptr_->index_udes_,
						mu+tmp_constraint_ptr_->index_mu_);
				//Initialize to tmp vector to zero
				for(int it_control=tmp_constraint_ptr_->index_u_; it_control<tmp_constraint_ptr_->index_u_+tmp_agent_ptr_->dim_u_; it_control++){
					out[it_control]+=dim_u_conc_[tmp_u];
				}
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_eq_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)
					&&((tmp_agent_ptr_->dim_u_>0)||(tmp_coupling_ptr_->agent2_->dim_u_>0))){
				double tmp_dcdu1mu[tmp_agent_ptr_->dim_u_];
				double tmp_dcdu2mu[tmp_coupling_ptr_->agent2_->dim_u_];
				tmp_coupling_ptr_->dcdumu(tmp_dcdu1mu,tmp_dcdu2mu,t,
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,
						u+tmp_coupling_ptr_->index_u1_,u+tmp_coupling_ptr_->index_u2_,
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_,
						udes+tmp_coupling_ptr_->index_udes1_,udes+tmp_coupling_ptr_->index_udes2_,
						mu+tmp_coupling_ptr_->index_mu_);
				//Save tmp to output and reset tmp
				for(int it_control=0; it_control<tmp_agent_ptr_->dim_u_; it_control++){
					out[tmp_coupling_ptr_->index_u1_+it_control]+=tmp_dcdu1mu[it_control];
				}
				for(int it_control=0; it_control<tmp_coupling_ptr_->agent2_->dim_u_; it_control++){
					out[tmp_coupling_ptr_->index_u2_+it_control]+=tmp_dcdu2mu[it_control];
				}
			}
		}
	}
}
/***** inequality constraint *****/
void Controller::ci		(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::ci()"<<std::endl;
#endif
	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->dim_ineq_>0){
				//Call function
				tmp_constraint_ptr_->ci(
						out+tmp_constraint_ptr_->index_mui_,
						t,
						x+tmp_constraint_ptr_->index_x_,
						u+tmp_constraint_ptr_->index_u_,
						p+tmp_constraint_ptr_->index_p_,
						p+tmp_constraint_ptr_->index_pc_,
						xdes+tmp_constraint_ptr_->index_xdes_,
						udes+tmp_constraint_ptr_->index_udes_);
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_ineq_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)){
				tmp_coupling_ptr_->ci(out+tmp_coupling_ptr_->index_mui_,t,
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,
						u+tmp_coupling_ptr_->index_u1_,u+tmp_coupling_ptr_->index_u2_,
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_,
						udes+tmp_coupling_ptr_->index_udes1_,udes+tmp_coupling_ptr_->index_udes2_);
			}
		}
	}
}
void Controller::dcidxmui	(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes, double *mui){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::dcidx_mui()"<<std::endl;
#endif
	double tmp_x[dim_x_conc_];
	memset(out,   0, dim_x_conc_*sizeof (double));
	memset(tmp_x, 0, dim_x_conc_*sizeof (double));
//	for(int it_state=0; it_state<dim_x_conc_; it_state++){tmp_x[it_state]=0;out[it_state]=0;};

	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->dim_ineq_>0&&tmp_constraint_ptr_->dim_x_>0){
				//Call function
				tmp_constraint_ptr_->dcidxmui(
						tmp_x+tmp_constraint_ptr_->index_x_,
						t,
						x+tmp_constraint_ptr_->index_x_,
						u+tmp_constraint_ptr_->index_u_,
						p+tmp_constraint_ptr_->index_p_,
						p+tmp_constraint_ptr_->index_pc_,
						xdes+tmp_constraint_ptr_->index_xdes_,
						udes+tmp_constraint_ptr_->index_udes_,
						mui+tmp_constraint_ptr_->index_mui_);
				//Initialize to tmp vector to zero
				for(int it_state=tmp_constraint_ptr_->index_x_; it_state<tmp_constraint_ptr_->index_x_+agentlist_[it_agent]->dim_x_; it_state++){
					out[it_state]+=tmp_x[it_state];
				}
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_ineq_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)
					&&((tmp_agent_ptr_->dim_x_>0)||(tmp_coupling_ptr_->agent2_->dim_x_>0))){
				double tmp_dcidx1mu[tmp_agent_ptr_->dim_x_];
				double tmp_dcidx2mu[tmp_coupling_ptr_->agent2_->dim_x_];
				tmp_coupling_ptr_->dcidxmui(tmp_dcidx1mu,tmp_dcidx2mu,t,
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,
						u+tmp_coupling_ptr_->index_u1_,u+tmp_coupling_ptr_->index_u2_,
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_,
						udes+tmp_coupling_ptr_->index_udes1_,udes+tmp_coupling_ptr_->index_udes2_,
						mui+tmp_coupling_ptr_->index_mui_);
				//Save tmp to output and reset tmp
				for(int it_state=0; it_state<tmp_agent_ptr_->dim_x_; it_state++){
					out[tmp_coupling_ptr_->index_x1_+it_state]+=tmp_dcidx1mu[it_state];
				}
				for(int it_state=0; it_state<tmp_coupling_ptr_->agent2_->dim_x_; it_state++){
					out[tmp_coupling_ptr_->index_x2_+it_state]+=tmp_dcidx2mu[it_state];
				}
			}
		}
	}
}
void Controller::dcidumui	(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes, double *mui){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::dcidu_mui()"<<std::endl;
#endif
	double tmp_u[dim_u_conc_];
	memset(out,   0, dim_u_conc_*sizeof (double));
	memset(tmp_u, 0, dim_u_conc_*sizeof (double));
//	for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;out[it_control]=0;};

	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->dim_ineq_>0&&tmp_constraint_ptr_->dim_u_>0){
				//Call function
				tmp_constraint_ptr_->dcidumui(
						tmp_u+tmp_constraint_ptr_->index_u_,
						t,
						x+tmp_constraint_ptr_->index_x_,
						u+tmp_constraint_ptr_->index_u_,
						p+tmp_constraint_ptr_->index_p_,
						p+tmp_constraint_ptr_->index_pc_,
						xdes+tmp_constraint_ptr_->index_xdes_,
						udes+tmp_constraint_ptr_->index_udes_,
						mui+tmp_constraint_ptr_->index_mui_);
				//Initialize to tmp vector to zero
				for(int it_control=tmp_constraint_ptr_->index_u_; it_control<tmp_constraint_ptr_->index_u_+tmp_agent_ptr_->dim_u_; it_control++){
					out[it_control]+=tmp_u[it_control];
				}
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_ineq_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)
					&&((tmp_agent_ptr_->dim_u_>0)||(tmp_coupling_ptr_->agent2_->dim_u_>0))){
				double tmp_dcidu1mui[tmp_agent_ptr_->dim_u_];
				double tmp_dcidu2mui[tmp_coupling_ptr_->agent2_->dim_u_];
				tmp_coupling_ptr_->dcidumui(tmp_dcidu1mui,tmp_dcidu2mui,t,
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,
						u+tmp_coupling_ptr_->index_u1_,u+tmp_coupling_ptr_->index_u2_,
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_,
						udes+tmp_coupling_ptr_->index_udes1_,udes+tmp_coupling_ptr_->index_udes2_,
						mui+tmp_coupling_ptr_->index_mui_);
				//Save tmp to output and reset tmp
				for(int it_control=0; it_control<tmp_agent_ptr_->dim_u_; it_control++){
					out[tmp_coupling_ptr_->index_u1_+it_control]+=tmp_dcidu1mui[it_control];
				}
				for(int it_control=0; it_control<tmp_coupling_ptr_->agent2_->dim_u_; it_control++){
					out[tmp_coupling_ptr_->index_u2_+it_control]+=tmp_dcidu2mui[it_control];
				}
			}
		}
	}
}


/***** inequality constraint *****/
void Controller::cia		(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes, double *slack){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::ci()"<<std::endl;
#endif
	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->dim_ineq_>0){
				//Call function
				tmp_constraint_ptr_->cia(
						out+tmp_constraint_ptr_->index_mui_,
						t,
						x+tmp_constraint_ptr_->index_x_,
						u+tmp_constraint_ptr_->index_u_,
						p+tmp_constraint_ptr_->index_p_,
						p+tmp_constraint_ptr_->index_pc_,
						xdes+tmp_constraint_ptr_->index_xdes_,
						udes+tmp_constraint_ptr_->index_udes_,
						slack+tmp_constraint_ptr_->index_mui_);
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_ineq_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)){
				tmp_coupling_ptr_->cia(out+tmp_coupling_ptr_->index_mui_,t,
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,
						u+tmp_coupling_ptr_->index_u1_,u+tmp_coupling_ptr_->index_u2_,
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_,
						udes+tmp_coupling_ptr_->index_udes1_,udes+tmp_coupling_ptr_->index_udes2_,slack+tmp_coupling_ptr_->index_mui_);
			}
		}
	}
}
void Controller::dciadxmui	(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes, double *mui, double *slack){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::dcidx_mui()"<<std::endl;
#endif
	double tmp_x[dim_x_conc_];
	memset(out,   0, dim_x_conc_*sizeof (double));
	memset(tmp_x, 0, dim_x_conc_*sizeof (double));
//	for(int it_state=0; it_state<dim_x_conc_; it_state++){tmp_x[it_state]=0;out[it_state]=0;};

	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->dim_ineq_>0&&tmp_constraint_ptr_->dim_x_>0){
				//Call function
				tmp_constraint_ptr_->dciadxmui(
						tmp_x+tmp_constraint_ptr_->index_x_,
						t,
						x+tmp_constraint_ptr_->index_x_,
						u+tmp_constraint_ptr_->index_u_,
						p+tmp_constraint_ptr_->index_p_,
						p+tmp_constraint_ptr_->index_pc_,
						xdes+tmp_constraint_ptr_->index_xdes_,
						udes+tmp_constraint_ptr_->index_udes_,
						mui+tmp_constraint_ptr_->index_mui_,
						slack+tmp_constraint_ptr_->index_mui_);
				//Initialize to tmp vector to zero
				for(int it_state=tmp_constraint_ptr_->index_x_; it_state<tmp_constraint_ptr_->index_x_+agentlist_[it_agent]->dim_x_; it_state++){
					out[it_state]+=tmp_x[it_state];
				}
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_ineq_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)
					&&((tmp_agent_ptr_->dim_x_>0)||(tmp_coupling_ptr_->agent2_->dim_x_>0))){
				double tmp_dcidx1mu[tmp_agent_ptr_->dim_x_];
				double tmp_dcidx2mu[tmp_coupling_ptr_->agent2_->dim_x_];
				tmp_coupling_ptr_->dciadxmui(tmp_dcidx1mu,tmp_dcidx2mu,t,
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,
						u+tmp_coupling_ptr_->index_u1_,u+tmp_coupling_ptr_->index_u2_,
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_,
						udes+tmp_coupling_ptr_->index_udes1_,udes+tmp_coupling_ptr_->index_udes2_,
						mui+tmp_coupling_ptr_->index_mui_,slack+tmp_coupling_ptr_->index_mui_);
				//Save tmp to output and reset tmp
				for(int it_state=0; it_state<tmp_agent_ptr_->dim_x_; it_state++){
					out[tmp_coupling_ptr_->index_x1_+it_state]+=tmp_dcidx1mu[it_state];
				}
				for(int it_state=0; it_state<tmp_coupling_ptr_->agent2_->dim_x_; it_state++){
					out[tmp_coupling_ptr_->index_x2_+it_state]+=tmp_dcidx2mu[it_state];
				}
			}
		}
	}
}
void Controller::dciadumui	(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes, double *mui, double *slack){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::dcidu_mui()"<<std::endl;
#endif
	double tmp_u[dim_u_conc_];
	memset(out,   0, dim_u_conc_*sizeof (double));
	memset(tmp_u, 0, dim_u_conc_*sizeof (double));
//	for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;out[it_control]=0;};

	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->dim_ineq_>0&&tmp_constraint_ptr_->dim_u_>0){
				//Call function
				tmp_constraint_ptr_->dciadumui(
						tmp_u+tmp_constraint_ptr_->index_u_,
						t,
						x+tmp_constraint_ptr_->index_x_,
						u+tmp_constraint_ptr_->index_u_,
						p+tmp_constraint_ptr_->index_p_,
						p+tmp_constraint_ptr_->index_pc_,
						xdes+tmp_constraint_ptr_->index_xdes_,
						udes+tmp_constraint_ptr_->index_udes_,
						mui+tmp_constraint_ptr_->index_mui_,
						slack+tmp_constraint_ptr_->index_mui_);
				//Initialize to tmp vector to zero
				for(int it_control=tmp_constraint_ptr_->index_u_; it_control<tmp_constraint_ptr_->index_u_+tmp_agent_ptr_->dim_u_; it_control++){
					out[it_control]+=tmp_u[it_control];
				}
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_ineq_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)
					&&((tmp_agent_ptr_->dim_u_>0)||(tmp_coupling_ptr_->agent2_->dim_u_>0))){
				double tmp_dcidu1mui[tmp_agent_ptr_->dim_u_];
				double tmp_dcidu2mui[tmp_coupling_ptr_->agent2_->dim_u_];
				tmp_coupling_ptr_->dciadumui(tmp_dcidu1mui,tmp_dcidu2mui,t,
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,
						u+tmp_coupling_ptr_->index_u1_,u+tmp_coupling_ptr_->index_u2_,
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_,
						udes+tmp_coupling_ptr_->index_udes1_,udes+tmp_coupling_ptr_->index_udes2_,
						mui+tmp_coupling_ptr_->index_mui_,slack+tmp_coupling_ptr_->index_mui_);
				//Save tmp to output and reset tmp
				for(int it_control=0; it_control<tmp_agent_ptr_->dim_u_; it_control++){
					out[tmp_coupling_ptr_->index_u1_+it_control]+=tmp_dcidu1mui[it_control];
				}
				for(int it_control=0; it_control<tmp_coupling_ptr_->agent2_->dim_u_; it_control++){
					out[tmp_coupling_ptr_->index_u2_+it_control]+=tmp_dcidu2mui[it_control];
				}
			}
		}
	}
}

void Controller::dciadamui	(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes, double *mui, double *slack){
#ifdef DEBUG_FUNCTION_TRACE
	std::cout<<"exec Controller::dcidu_mui()"<<std::endl;
#endif
	double tmp_u[dim_u_conc_];
	memset(out,   0, dim_u_conc_*sizeof (double));
	memset(tmp_u, 0, dim_u_conc_*sizeof (double));
//	for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;out[it_control]=0;};

	for(int it_agent=0;it_agent<agentlist_.size();it_agent++){
		tmp_agent_ptr_=agentlist_[it_agent];
		for(int it_constraint=0;it_constraint<tmp_agent_ptr_->getConstraint_Dim();it_constraint++){
			tmp_constraint_ptr_=tmp_agent_ptr_->constraint_[it_constraint];
			if(tmp_constraint_ptr_->dim_ineq_>0&&tmp_constraint_ptr_->dim_u_>0){
				//Call function
				tmp_constraint_ptr_->dciadamui(
						out+tmp_constraint_ptr_->index_mui_,
						t,
						x+tmp_constraint_ptr_->index_x_,
						u+tmp_constraint_ptr_->index_u_,
						p+tmp_constraint_ptr_->index_p_,
						p+tmp_constraint_ptr_->index_pc_,
						xdes+tmp_constraint_ptr_->index_xdes_,
						udes+tmp_constraint_ptr_->index_udes_,
						mui+tmp_constraint_ptr_->index_mui_,
						slack+tmp_constraint_ptr_->index_mui_);
				//Initialize to tmp vector to zero
			}
		}
		//Loop over couplings in agents
		for(int it_coupling=0;it_coupling<tmp_agent_ptr_->getCoupling_Dim();it_coupling++){
			tmp_coupling_ptr_=tmp_agent_ptr_->coupling_[it_coupling];
			//Check if agent is first coupling agent and stage cost existent
			if(tmp_coupling_ptr_->dim_ineq_>0&&(tmp_coupling_ptr_->agent1_==tmp_agent_ptr_)
					&&(tmp_agent_ptr_->dim_ineq_>0)){
				tmp_coupling_ptr_->dciadamui(out+tmp_coupling_ptr_->index_mui_,t,
						x+tmp_coupling_ptr_->index_x1_,x+tmp_coupling_ptr_->index_x2_,
						u+tmp_coupling_ptr_->index_u1_,u+tmp_coupling_ptr_->index_u2_,
						p+tmp_coupling_ptr_->index_p1_,p+tmp_coupling_ptr_->index_p2_,p+tmp_coupling_ptr_->index_pc_,
						xdes+tmp_coupling_ptr_->index_xdes1_,xdes+tmp_coupling_ptr_->index_xdes2_,
						udes+tmp_coupling_ptr_->index_udes1_,udes+tmp_coupling_ptr_->index_udes2_,
						mui+tmp_coupling_ptr_->index_mui_,slack+tmp_coupling_ptr_->index_mui_);
			}
		}
	}
}

//void Controller::dfdx			(double **out,double t,double *x, double *u, double *p){
//#ifdef DEBUG_FUNCTION_TRACE
//	std::cout<<"exec Controller::dfdx()"<<std::endl;
//#endif
//}
//void Controller::dfdu			(double **out,double t,double *x, double *u, double *p){
//#ifdef DEBUG_FUNCTION_TRACE
//	std::cout<<"exec Controller::dfdu()"<<std::endl;
//#endif
//}
//void Controller::dcdx		(double **out,double t,double *x, double *u, double *p, double *xdes, double *udes){
//#ifdef DEBUG_FUNCTION_TRACE
//	std::cout<<"exec Controller::dcdx()"<<std::endl;
//#endif
//}
//void Controller::dcdu		(double **out,double t,double *x, double *u, double *p, double *xdes, double *udes){
//#ifdef DEBUG_FUNCTION_TRACE
//	std::cout<<"exec Controller::dcdu()"<<std::endl;
//#endif
//}
//void Controller::dcidx	(double **out,double t, double *x, double *u, double *p, double *xdes, double *udes){
//#ifdef DEBUG_FUNCTION_TRACE
//	std::cout<<"exec Controller::dcidx()"<<std::endl;
//#endif
//}
//void Controller::dcidu	(double **out,double t, double *x, double *u, double *p, double *xdes, double *udes){
//#ifdef DEBUG_FUNCTION_TRACE
//	std::cout<<"exec Controller::dcidu()"<<std::endl;
//#endif
//}






//void Controller::calculateActuation(double time){
//	unsigned i=0,j=0,l=0;
//	//Temporary variable to store control output
//	double clock;
//	//Loop over all controllers
//	for(l=0; l<1/*this->controllist.size()*/;l++ ){
//		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//		// No indexing implemented for multiple controllers
//		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//#ifdef CONTROLLER_INFO
//		MathLib::mprint(this->state,"Agent::calculateActuation():in=state");
//#endif
//
//		for(j=0;j<this->agentlist_.size();j++){
//			//Get Desired Actuation from Agents
//			agentlist_[j]->getDesiredActuation	(&desiredactuation[actuation_indices[j]]);
//			//Get Desired State from Agents
//			agentlist_[j]->getDesiredState	(&desiredstate[state_indices[j]]);
//			//Get Timevarpar from Agents
//			agentlist_[j]->getTimevarpar		(&timevarpar[timevarpar_indices[j]]);
//		}
//
//		//Set controller actuation as plant output
//		this->controllist[l]->setState(state);
//		//Set controller desired actuation as plant desired output
//		this->controllist[l]->setDesiredState(desiredstate);
//		//Set controller output as plant desired output
//		this->controllist[l]->setDesiredOutput(desiredactuation);
//		// Calculate the controller output which equals to the plants actuation
//		clock=ros::Time::now().toSec();
//		this->controllist[l]->calculateOutput(time);
//		clock=ros::Time::now().toSec()-clock;
//		//Get controller output
//		controllist[l]->getOutput(&(actuation[0]));
//
//		//Set Actuation to agents
//		for(j=0;j<this->agentlist_.size();j++){
//			agentlist_[j]->setVectorToActuation(&(actuation[actuation_indices[j]]));
//		}
//		//If saving activated
//		if(saveflag){
//			//Save agent data
//			save(time,clock);
//		}
//
//	}
//}
//
//void Controller::calculateState(){
//#ifdef DEBUG_FUNCTION_TRACE
//	printf("composer calculateState\n");
//#endif
//	for(int j=0;j<this->agentlist_.size();j++){
//		//Get State from Agents
//		agentlist_[j]->getState	(&state[state_indices[j]]);
//	}
//}
//
//
//void Controller::print(){
//#ifdef DEBUG_FUNCTION_TRACE
//	printf("composer print\n");
//#endif
//	printf("------------------------------------------------------------------------------");
//	printf("-----------------------------------------------------------------------------\n");
//	printf("%s :",this->type.c_str());
//	printf(" id_:%i\n",this->id_);
//	if(!controllist.empty()){
//		printf("- %s",this->controllist[0]->getType().c_str());
//		printf(" id_:%i\n",this->controllist[0]->getId());
//	}
//	//Print states
//	printf("        state:");
//	for(int i=0;i<dim_state;i++){
//		printf("%+8.4f,",state[i]);
//	}
//	//Print desired states
//	printf("\ndesired state:");
//	for(int i=0;i<dim_state;i++){
//		printf("%+8.4f,",desiredstate[i]);
//	}
//	printf("\n");
//}
//
//Controller::Controller(int id_, std::vector<Agent*> listofagents):Agent(id_)
//{
//	int i,j,k;
//
//	this->id_=id_;
//	//Set Type
//	this->type="Controller";
//	printf("Agent%i",this->id_);
//	printf(" of type %s created\n",type.c_str());
//	this->agentlist_=listofagents;
//
//	//COMPOSE COUPLINGS
//	//Determine Couplings of the Agents
//	std::vector<Coupling*> tmp_edges;
//	for(i=0;i<this->agentlist_.size();i++){
//		//Store edges of an agent in tmp_edges
//		tmp_edges=agentlist_[i]->getEdgeList();
//		//Store edges of an agent to edges
//		edges.insert(edges.end(),tmp_edges.begin(),tmp_edges.end());
//	}
//	//Remove Dublicated Couplings
//	std::sort( edges.begin(), edges.end() );
//	edges.erase( std::unique( edges.begin(), edges.end() ), edges.end() );
//	//Get the indices of coupled agents in the agentlist_ to be able to map
//	//the corresponding states and actuations to the coupling system functions
//	if(edges.size()>0){
//		edge_indices=MathLib::def_int_matrix(edges.size(),2,"Edge indeces");
//		for(int j=0;j<edges.size();j++){
//			for(int i=0;i<agentlist_.size();i++){
//				if(edges[j]->getAgenti()==agentlist_[i]){
//					edge_indices[j][0]=i;
//				}
//				else if(edges[j]->getAgentj()==agentlist_[i]){
//					edge_indices[j][1]=i;
//				}
//			}
//		}
//	}
//	//Initialize the dimensions
//	dim_actuation=0;
//	dim_state=0;
//	dim_timevarpar=0;
//	dim_equalityconstraint=0;
//	dim_inequalityconstraint=0;
//	//Get total vector dimenstions
//	for(i=0;i<this->agentlist_.size();i++){
//		dim_actuation+=agentlist_[i]->getActuationDimension();
//		dim_state+=agentlist_[i]->getStateDimension();
//		dim_timevarpar+=agentlist_[i]->getTimevarparDimension();
//		dim_equalityconstraint+=agentlist_[i]->getEqualityConstraintDimension();
//		dim_inequalityconstraint+=agentlist_[i]->getInequalityConstraintDimension();
//	}
//	for(i=0;i<this->edges.size();i++){
//		dim_equalityconstraint+=edges[i]->getEqualityConstraintDimension();
//		dim_inequalityconstraint+=edges[i]->getInequalityConstraintDimension();
//	}
//
//	//Allocate index vectors (+one to put total dimension at the end)
//	actuation_indices			=MathLib::def_int_vector(dim_actuation+1,"actuation_indices");
//	state_indices				=MathLib::def_int_vector(dim_state+1,"state_indices");
//	timevarpar_indices			=MathLib::def_int_vector(dim_timevarpar+1,"timevarpar_indices");
//	inequalityconstraint_indices	=MathLib::def_int_vector(dim_inequalityconstraint+1,"inequalityconstraint_indices");
//	equalityconstraint_indices			=MathLib::def_int_vector(dim_equalityconstraint+1,"equalityconstraint_indices");
//	//Introduce temporary variables
//	tmp_actuation	=MathLib::defvector(dim_actuation,"actuation_indices");
//	tmp_state		=MathLib::defvector(dim_state,"tmp_state");
//
//	//Initialize index vectors
//	actuation_indices[0]=0;
//	state_indices[0]=0;
//	timevarpar_indices[0]=0;
//	equalityconstraint_indices[0]=0;
//	inequalityconstraint_indices[0]=0;
//
//	//COMPOSE THE VECTORS FROM AGENTS
//	//Store the indeces of the vector place in the composed vector
//	for(i=0;i<this->agentlist_.size();i++){
//		actuation_indices[i+1]=actuation_indices[i]+agentlist_[i]->getActuationDimension();
//		state_indices[i+1]=state_indices[i]+agentlist_[i]->getStateDimension();
//		timevarpar_indices[i+1]=timevarpar_indices[i]+agentlist_[i]->getTimevarparDimension();
//		inequalityconstraint_indices[i+1]=inequalityconstraint_indices[i]+agentlist_[i]->getInequalityConstraintDimension();
//		equalityconstraint_indices[i+1]=equalityconstraint_indices[i]+agentlist_[i]->getEqualityConstraintDimension();
//	}
//	for(j=0;i<this->edges.size();j++,i++){
//		inequalityconstraint_indices[i+1]=inequalityconstraint_indices[i]+edges[j]->getInequalityConstraintDimension();
//		equalityconstraint_indices[i+1]=equalityconstraint_indices[i]+edges[j]->getEqualityConstraintDimension();
//	}
//
//
//	printf("Concatenate actuation vector of      %5u elements\n",dim_actuation);
//	printf("Concatenate state vector of      %5u elements\n",dim_state);
//	//	printf("Concatenate output vector of     %5u elements\n",dim_output);
//	printf("Concatenate timevarpar vector of %5u elements\n",dim_timevarpar);
//	printf("Concatenate equality constraint vector of %5u elements\n",dim_equalityconstraint);
//	printf("Concatenate inequality constraint vector of %5u elements\n",dim_inequalityconstraint);
//
//	this->actuation.		resize(dim_actuation,0);
//	this->desiredactuation.	resize(dim_actuation,0);
//	this->state.		resize(dim_state,0);
//	this->desiredstate.	resize(dim_state,0);
//	this->timevarpar.	resize(dim_timevarpar,0);
//
//	for(j=0;j<this->agentlist_.size();j++){
//		//Get Actuation from Agents
//		agentlist_[j]->getActuation			(&actuation[actuation_indices[j]]);
//		//Get Desired Actuation from Agents
//		agentlist_[j]->getDesiredActuation	(&desiredactuation[actuation_indices[j]]);
//		//Get State from Agents
//		agentlist_[j]->getState			(&state[state_indices[j]]);
//		//Get Desired State from Agents
//		agentlist_[j]->getDesiredState	(&desiredstate[state_indices[j]]);
//		//Get Timevarpar from Agents
//		agentlist_[j]->getTimevarpar		(&timevarpar[timevarpar_indices[j]]);
//	}
//
//	//If datasaving is on
//	if(saveflag){
//		initSave();
//	}
//}

