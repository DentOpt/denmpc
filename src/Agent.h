/**
 * @file    Agent.h
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

#ifndef _AGENT_H
#define _AGENT_H

//#define DEBUG_CGMRES
//#define DEBUG_FUNCTION_TRACE
//#define AGENT_INFO

#define INPUTDATA_CHECK

//#include "Coupling.h"
#include <iostream>
#include <vector>	//For vector
#include <string>	//For string
#include <ros/ros.h>//For vector
#include <std_msgs/Float32.h>			//For String Messages
#include "MathLib.h"
#include "Constraint.h"
#include "Controller.h"
#include "Indexing.h"

using namespace MathLib;

//Forward Declaration
class Constraint;
class Coupling;
class Controller;

class Agent:public AgentIndex{
	friend class Controller;
protected:
	//=================================
	//Variables
	int id_;					//ID of the agent
	std::string  name_;		//Name of the agent
	bool 	flag_record_;	//Flag for recording
	bool 	flag_display_;	//Flag for display
	//States
	double* x_;		//States
	double* u_;		//Controls
	double* y_;		//Outputs
	double* d_;		//Disturbance
	double* xdes_;	//Desired States
	double* udes_;	//Desired Controls
	double* ydes_;	//Desired Outputs
	double* p_;		//Parameters

	//Limits
//	double* x_limu_;	//States upper limit
//	double* u_limu_;	//Controls upper limit
//	double* y_limu_;	//Outputs upper limit
//	double* x_liml_;	//States lower limit
//	double* u_liml_;	//Controls lower limit
//	double* y_liml_;	//Outputs lower limit
	//Initial States
	double* x_init_;		//States
	double* u_init_;		//Controls
	double* y_init_;		//Outputs
	double* d_init_;		//Disturbance
	double* xdes_init_;	//Desired States
	double* udes_init_;	//Desired Controls
	double* ydes_init_;	//Desired Outputs
	double* p_init_;		//Parameters
	//State Dimensions
	int 	dim_x_;
	int 	dim_u_;
	int 	dim_y_;
	int 	dim_d_;
	int 	dim_xdes_;
	int 	dim_udes_;
	int 	dim_ydes_;
	int 	dim_p_;
	int 	dim_ineq_;
	int 	dim_eq_;
	int 	dim_l_;
	int 	dim_v_;
	//Flags
	bool 	saveflag_;
	bool 	printflag_;
	bool 	plotflag_;
	//Handles
	ros::NodeHandle 		ros_node_;
	std::vector<ros::Publisher*>  ros_publishers_;
	std::vector<ros::Subscriber*> ros_state_subscribers_;
	std::vector<ros::Subscriber*> ros_desired_state_subscribers_;
	std::vector<ros::Subscriber*> ros_control_subscribers_;
	std::vector<ros::Subscriber*> ros_desired_control_subscribers_;
	std::vector<ros::Subscriber*> ros_parameter_subscribers_;
	//Vector of constraints

	std::vector<Constraint*> constraint_;
	std::vector<Coupling*>   coupling_;
	std::vector<Controller*> controller_;


	inline void setVector(int dim, double* vector,double* out,const char* error){
		for(int i=0;i<dim;i++){
#ifdef INPUTDATA_CHECK
			//CHECK IF DATA IS NAN => SET TO 0
			if(isnan(vector[i])){
				out[i]=0;
				std::cout<<"!!! In "<<name_<<": "<<error<<"["<<i<<"] ERROR NAN DETECTED!"<<std::endl;
			}
			else
#endif
			{
				out[i]=vector[i];
			}
		}
	}
	inline void setVector(int dim, std::vector<double> vector,double* out,const char* error){
		for(int i=0;i<dim;i++){
#ifdef INPUTDATA_CHECK
			//CHECK IF DATA IS NAN => SET TO 0
			if(isnan(vector[i])){
				out[i]=0;
				std::cout<<"!!! In "<<name_<<": "<<error<<"["<<i<<"] ERROR NAN DETECTED!"<<std::endl;
			}
			else
#endif
			{
				out[i]=vector[i];
			}
		}
	}


public: 
	//Constructors
	Agent();
	Agent(int _id);
	//	Agent(int _id,
	//			std::vector<double> init_x_,
	//			std::vector<double> init_xdes_,
	//			std::vector<double> init_u_,
	//			std::vector<double> init_udes_,
	//			std::vector<double> init_y_,
	//			std::vector<double> init_ydes_,
	//			std::vector<double> init_p_);


	~Agent();

	//Getter Methods
	int 		getId()		{return this->id_;};
	std::string getName()	{return this->name_;};
	inline void getState(double* _vector)			{for(int i=0;i<dim_x_;i++){_vector[i]=x_[i];}};
	inline void getControl(double* _vector)		  	{for(int i=0;i<dim_u_;i++){_vector[i]=u_[i];}};
	inline void getOutput(double* _vector)			{for(int i=0;i<dim_y_;i++){_vector[i]=y_[i];}};
	inline void getDisturbance(double* _vector)		{for(int i=0;i<dim_d_;i++){_vector[i]=d_[i];}};
	inline void getDesiredState(double* _vector)	{for(int i=0;i<dim_xdes_;i++){_vector[i]=xdes_[i];}};
	inline void getDesiredControl(double* _vector)	{for(int i=0;i<dim_udes_;i++){_vector[i]=udes_[i];}};
	inline void getDesiredOutput(double* _vector)	{for(int i=0;i<dim_ydes_;i++){_vector[i]=ydes_[i];}};
	inline void getParameter(double* _vector)		{for(int i=0;i<dim_p_;i++){_vector[i]=p_[i];}};

	inline void getState(std::vector<double>& _vector)	  		{for(int i=0;i<dim_x_;i++){_vector[i]=x_[i];}};
	inline void getControl(std::vector<double>& _vector)  		{for(int i=0;i<dim_u_;i++){_vector[i]=u_[i];}};
	inline void getOutput(std::vector<double>& _vector)			{for(int i=0;i<dim_y_;i++){_vector[i]=y_[i];}};
	inline void getDesiredState(std::vector<double>& _vector)	{for(int i=0;i<dim_xdes_;i++){_vector[i]=xdes_[i];}};
	inline void getDesiredControl(std::vector<double>& _vector)	{for(int i=0;i<dim_udes_;i++){_vector[i]=udes_[i];}};
	inline void getDesiredOutput(std::vector<double>& _vector)	{for(int i=0;i<dim_ydes_;i++){_vector[i]=ydes_[i];}};
	inline void getParameter(std::vector<double>& _vector)		{for(int i=0;i<dim_p_;i++){_vector[i]=p_[i];}};

	inline double* getState_Ptr()  			{return x_;};
	inline double* getControl_Ptr()			{return u_;};
	inline double* getOutput_Ptr() 			{return y_;};
	inline double* getDisturbance_Ptr()		{return d_;};
	inline double* getDesiredState_Ptr()  	{return xdes_;};
	inline double* getDesiredControl_Ptr()	{return udes_;};
	inline double* getDesiredOutput_Ptr() 	{return ydes_;};
	inline double* getParameter_Ptr()		{return p_;};

	inline int getState_Dim()  			{return dim_x_;};
	inline int getControl_Dim()			{return dim_u_;};
	inline int getOutput_Dim() 			{return dim_y_;};
	inline int getDisturbance_Dim()		{return dim_d_;};
	inline int getDesiredState_Dim()  	{return dim_xdes_;};
	inline int getDesiredControl_Dim()	{return dim_udes_;};
	inline int getDesiredOutput_Dim() 	{return dim_ydes_;};
	inline int getParameter_Dim()		{return dim_p_;};

	inline int getConstraint_Dim(){return constraint_.size();};
	inline Constraint* getConstraint(int index){return constraint_[index];};
	inline int getCoupling_Dim(){return coupling_.size();};
	inline Coupling* getCoupling(int index){return coupling_[index];};


	//Setter Methods
	int 		setId(int _id)	 		{id_=_id;};
	std::string setName(std::string _name)	{name_=_name;};
	inline void setControl(double* vector)		    {setVector(dim_u_,vector,u_,"Control");};
	inline void setState(double* vector)			{setVector(dim_x_,vector,x_,"State");};
	inline void setOutput(double* vector)			{setVector(dim_y_,vector,y_,"Output");};
	inline void setDisturbance(double* vector)		{setVector(dim_d_,vector,d_,"Disturbance");};
	inline void setDesiredControl(double* vector)	{setVector(dim_udes_,vector,udes_,"Desired Control");};
	inline void setDesiredState(double* vector)		{setVector(dim_xdes_,vector,xdes_,"Desired State");};
	inline void setDesiredOutput(double* vector)	{setVector(dim_ydes_,vector,ydes_,"Desired Output");};
	inline void setParameter(double* vector)		{setVector(dim_p_,vector,p_,"Parameter");};

	inline void setControl(std::vector<double> vector) 			{setVector(dim_u_,vector,u_,"Control");};
	inline void setState(std::vector<double> vector)	 		{setVector(dim_x_,vector,x_,"State");};
	inline void setOutput(std::vector<double> vector)	 		{setVector(dim_y_,vector,y_,"Output");};
	inline void setDisturbance(std::vector<double> vector)	 	{setVector(dim_d_,vector,d_,"Disturbance");};
	inline void setDesiredControl(std::vector<double> vector)	{setVector(dim_udes_,vector,udes_,"Desired Control");};
	inline void setDesiredState(std::vector<double> vector)		{setVector(dim_xdes_,vector,xdes_,"Desired State");};
	inline void setDesiredOutput(std::vector<double> vector)	{setVector(dim_ydes_,vector,ydes_,"Desired Output");};
	inline void setParameter(std::vector<double> vector)		{setVector(dim_p_,vector,p_,"Parameter");};

	inline void setInitialControl(double* vector)		{setVector(dim_u_,vector,u_init_,"Initial Control");};
	inline void setInitialState(double* vector)			{setVector(dim_x_,vector,x_init_,"Initial State");};
	inline void setInitialOutput(double* vector)		{setVector(dim_y_,vector,y_init_,"Initial Output");};
	inline void setInitialDisturbance(double* vector)	{setVector(dim_d_,vector,d_init_,"Initial Disturbance");};
	inline void setInitialDesiredControl(double* vector){setVector(dim_udes_,vector,udes_init_,"Initial Desired Control");};
	inline void setInitialDesiredState(double* vector)	{setVector(dim_xdes_,vector,xdes_init_,"Initial Desired State");};
	inline void setInitialDesiredOutput(double* vector)	{setVector(dim_ydes_,vector,ydes_init_,"Initial Desired Output");};
	inline void setInitialParameter(double* vector)		{setVector(dim_p_,vector,p_init_,"Initial Parameter");};

	inline void setInitialControl(std::vector<double> vector) 		{setVector(dim_u_,vector,u_init_,"Initial Control");};
	inline void setInitialState(std::vector<double> vector)	 		{setVector(dim_x_,vector,x_init_,"Initial State");};
	inline void setInitialOutput(std::vector<double> vector)	 	{setVector(dim_y_,vector,y_init_,"Initial Output");};
	inline void setInitialDisturbance(std::vector<double> vector)	{setVector(dim_d_,vector,d_init_,"Initial Disturbance");};
	inline void setInitialDesiredControl(std::vector<double> vector){setVector(dim_udes_,vector,udes_init_,"Initial Desired Control");};
	inline void setInitialDesiredState(std::vector<double> vector)	{setVector(dim_xdes_,vector,xdes_init_,"Initial Desired State");};
	inline void setInitialDesiredOutput(std::vector<double> vector)	{setVector(dim_ydes_,vector,ydes_init_,"Initial Desired Output");};
	inline void setInitialParameter(std::vector<double> vector)		{setVector(dim_p_,vector,p_init_,"Initial Parameter");};

	//Methods
	/*
	virtual void record(){};
	virtual void print(){};
	virtual void start(){printf("Agent%i start: not defined",this->id);};
	virtual void pause(){printf("Agent%i pause: not defined",this->id);};
	virtual void stop() {printf("Agent%i stop:  not defined",this->id);};
	// calculate plant actuation from plant output with controller	-> Call Controller
	virtual void calculateActuation(double time){std::cout<<"calculateActuation"<<std::endl;};
	// calculate plant output from measurement with Sensor	-> Call Sensor/Observer
	virtual void calculateState(){std::cout<<"calculateState"<<std::endl;};
	// simulate state
	virtual void simulateState(double time,double updateintervall){std::cout<<"simulateState"<<std::endl;};
	 */
	void record(){};
	void print(){};
	virtual void start(){printf("Agent%i start: ynot defined",this->id_);};
	virtual void pause(){printf("Agent%i pause: not defined",this->id_);};
	virtual void stop() {printf("Agent%i stop:  not defined",this->id_);};
	// calculate plant actuation from plant output with controller	-> Call Controller
	void calculateActuation(double time){std::cout<<"Agent::calculateActuation"<<std::endl;};
	// calculate plant output from measurement with Sensor	-> Call Sensor/Observer
	void calculateState(){std::cout<<"Agent::calculateState"<<std::endl;};
	// calculate plant output from measurement with Sensor	-> Call Sensor/Observer
//	void calculateDisturbance(double t){
//		double tmp_d[dim_d_];
//		//Estimate disturbance
//		d(tmp_d,t,x_,u_,d_,p_,xdes_,udes_);
//		//set result of d as new disturbance
//		setVector(dim_d_,tmp_d,d_,"calculateDisturbance():d");
//	};
//	// simulate state
//	void simulateState(double time,double updateintervall){std::cout<<"Agent::simulateState"<<std::endl;};
	//Publish Actuation
	virtual void rosPublishActuation(){std::cout<<"Agent::rosPublishActuation"<<std::endl;};
	//Add and remove Constraint
	void addConstraint(Constraint* _constraint);
	void removeConstraint(Constraint* _constraint);
	//Add and remove Constraint
	void addCouplingAsAgent1(Coupling* _coupling);
	void addCouplingAsAgent2(Coupling* _coupling);
	void removeCoupling(Coupling* _couplingconstraint);
	//Add and remove Controller
	void addController(Controller* _controller);
	void removeController(Controller* _controller);
	//Set initial states to states
	void reset2initialstate();


	/****** FUNCTIONS THAT ARE GENERATED WITH CODE GENERATION *****/
	virtual void d			(double  *out,double t, double *x, double *u, double *d, double *p, double *xdes, double *udes){std::cout<<"Agent "<<id_<<" "<<name_<<"::d not defined!!!"<<std::endl;}
	virtual void f			(double  *out,double t, double *x, double *u, double *d, double *p){std::cout<<"Agent "<<id_<<" "<<name_<<"::f not defined!!!"<<std::endl;}
	virtual void dfdx		(double **out,double t, double *x, double *u, double *d, double *p){std::cout<<"Agent "<<id_<<" "<<name_<<"::dfdx not defined!!!"<<std::endl;}
	virtual void dfdu		(double **out,double t, double *x, double *u, double *d, double *p){std::cout<<"Agent "<<id_<<" "<<name_<<"::dfdu not defined!!!"<<std::endl;}
	virtual void dfdxlambda	(double  *out,double t, double *x, double *u, double *d, double *p, double *lambda){std::cout<<"Agent "<<id_<<" "<<name_<<"::dfdxlambda not defined!!!"<<std::endl;}
	virtual void dfdulambda	(double  *out,double t, double *x, double *u, double *d, double *p, double *lambda){std::cout<<"Agent "<<id_<<" "<<name_<<"::dfdulambda not defined!!!"<<std::endl;}
	virtual void l			(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes){std::cout<<"Agent "<<id_<<" "<<name_<<"::l not defined!!!"<<std::endl;};
	virtual void dldx		(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes){std::cout<<"Agent "<<id_<<" "<<name_<<"::dldx not defined!!!"<<std::endl;};
	virtual void dldu		(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes){std::cout<<"Agent "<<id_<<" "<<name_<<"::dldu not defined!!!"<<std::endl;};
	virtual void v			(double  *out,double t, double *x, double *p, double *xdes){std::cout<<"Agent "<<id_<<" "<<name_<<"::v not defined!!!"<<std::endl;};
	virtual void dvdx		(double  *out,double t, double *x, double *p, double *xdes){std::cout<<"Agent "<<id_<<" "<<name_<<"::dvdx not defined!!!"<<std::endl;};

};

#endif //_AGENT_H
