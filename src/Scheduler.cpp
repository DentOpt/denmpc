/**
 * @file    Scheduler.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Scheduler Container
 *
 * Scheduler is managing the communication and control update intervalls.
 * It also allows switching between control, simulation and V-Rep mode.
 */

#include "Scheduler.h"

/**
 * Scheduler implementation
 */
//Scheduler::Scheduler(int argc, char **argv, vector<Agent*> _agentlist, vector<Constraint*> constraintlist){
Scheduler::Scheduler(
		int argc,
		char **argv,
		std::vector<Controller*> controllerlist,
		std::vector<Event*> eventlist){
	//Store agentlist_
	controllerlist_=controllerlist;
	eventlist_	   =eventlist;

	//Initialize ROS
	ros::init(argc, argv, "fccf");
	//Printing starting screen
	printf("//*********************************************\n");
	printf("// Starting Fast Cooperative Control Framework \n");
	printf("//*********************************************\n");

	//Initialization
	//Printing initialization screen
	printf("// Initialization \n");
	printf("//---------------------------------------------\n");

}

void Scheduler::controlloop(){
	//Get systemtime of framework start [s]
	ros_time_calibration_=ros::Time::now().toSec();
	//Set the update loop intervall [1/s]
	ros::Rate loop_frequency(1/time_intervall_);
	//Index
	unsigned i;
	//Printing loop start screen
	printf("//---------------------------------------------\n");
	printf("// Start ROS Control Loop \n");


	//Starting Agents
	for(int it_controller=0;it_controller<controllerlist_.size();it_controller++){
		ptr_controller=controllerlist_[it_controller];
		ptr_controller->startAgents();
	}


	//Starting fccf loop
	//--------------------------------------------------------------
	for(counter_fccfloop_=0;ros::ok();counter_fccfloop_++){
		//Calculate system time in framework
		time_framework_=ros::Time::now().toSec()-ros_time_calibration_;
		//Print time in framework
		std::cout<<"/*************************************************************************\\"<<std::endl;
		std::cout<<"/* ";
		printf("loop: %7i ",counter_fccfloop_);
		printf("time: %7.4f[s]",time_framework_);
		std::cout<<"                                        *\\"<<std::endl;
		std::cout<<"/*************************************************************************\\"<<std::endl;

		//Check if event happend
		bool flag_event_happened=false;
		for(int it_event=0;it_event<eventlist_.size();it_event++){
			if(eventlist_[it_event]->check(time_framework_)){
				flag_event_happened=true;
			}
		}
		//Reinit if event happened
		if(flag_event_happened){
			flag_event_happened=false;
			for(int it_controller=0;it_controller<controllerlist_.size();it_controller++){
				controllerlist_[it_controller]->init();
			}
		}
		//Control loop
		for(int it_controller=0;it_controller<controllerlist_.size();it_controller++){
			ptr_controller=controllerlist_[it_controller];
			ptr_controller->getMeasurements();
			ptr_controller->computeAction(time_framework_);
			ptr_controller->applyAction();
		}

		//Sleep until endtime of loop intervall
		loop_frequency.sleep();
		//Run ROS Processes
		ros::spinOnce();
	}
	printf("//**********************************************\n");
	printf("// Finishing Fast Cooperative Control Framework \n");
	printf("//**********************************************\n");

}
void Scheduler::vrepsimrunning(const dentopt_nmpc_controller::VrepInfo& msgInfo){
	//	printf("\n%u\n",msgInfo.simulatorState.data);
	int info=msgInfo.simulatorState.data;
	bool old_vrep_flag_stop =vrep_flag_stop_;
	bool old_vrep_flag_pause=vrep_flag_pause_;
	//Bit 0 True for simulation not stopped
	//Bit 1 True for simulation paused
	vrep_flag_stop_=!(info&0b001);
	vrep_flag_pause_=(info&0b010);
	vrep_flag_realtime_=(info&0b100);
	vrep_time_=msgInfo.simulationTime.data;
	//printf("Vrep Flag: Stop:%d Pause:%d\n",vrep_flag_stop_,vrep_flag_pause_);
	if((vrep_flag_stop_)&&(!old_vrep_flag_stop)){
		printf( "!!!!!!!!!!!!!!!!!!\n");
		printf( "!! vrep stopped !!\n");
		printf( "!!!!!!!!!!!!!!!!!!\n");
	}
	if((vrep_flag_pause_)&&(!old_vrep_flag_pause)){
		printf( "!!!!!!!!!!!!!!!!!\n");
		printf( "!! vrep paused !!\n");
		printf( "!!!!!!!!!!!!!!!!!\n");
	}
}
void Scheduler::vrepcontrolloop(){
	//Get Vrep simulation
	ros::Subscriber rosSubscriberVrepInfo;
	ros::NodeHandle rosNodeVrepInfo;
	//Get info about Vrep Simulation
	rosSubscriberVrepInfo = rosNodeVrepInfo.subscribe(rosNodeVrepInfo.resolveName("vrep/info"),1,&Scheduler::vrepsimrunning,this);
	//Printing loop start screen
		printf("//---------------------------------------------\n");
		printf("// Start VREP Control Loop \n");
	ros::Duration(0.2).sleep(); //Wait for subscriber
	ros::spinOnce(); //Check Vrep state
	//Set the update loop intervall [1/s]
	ros::Rate loop_frequency(1/time_intervall_);
	//Index
	unsigned i;



	//Starting Agents
	for(int it_controller=0;it_controller<controllerlist_.size();it_controller++){
		ptr_controller=controllerlist_[it_controller];
		ptr_controller->startAgents();
	}



	//Starting fccf loop
	//--------------------------------------------------------------
	for(counter_fccfloop_=0;ros::ok();counter_fccfloop_++){
		//Wait if simulation is stopped
		while(ros::ok()&&((vrep_flag_stop_==true)||(vrep_flag_pause_==true))){
			ros::spinOnce();
		};
		printf("//---------------------------------------------\n");
		//Calculate system time in framework
		time_framework_=vrep_time_;
		//Print time in framework
		printf("loop: %7i ",counter_fccfloop_);
		printf("time: %7.4f\n",time_framework_);
		double tmp_start=ros::Time::now().toSec();

		//Check if event happend
		bool flag_event_happened=false;
		for(int it_event=0;it_event<eventlist_.size();it_event++){
			if(eventlist_[it_event]->check(time_framework_)){
				flag_event_happened=true;
			}
		}
		//Reinit if event happened
		if(flag_event_happened){
			flag_event_happened=false;
			for(int it_controller=0;it_controller<controllerlist_.size();it_controller++){
				controllerlist_[it_controller]->init();
			}
		}

		//Control loop
		for(int it_controller=0;it_controller<controllerlist_.size();it_controller++){
			ptr_controller=controllerlist_[it_controller];
			ptr_controller->getMeasurements();
			ptr_controller->computeAction(time_framework_);
			ptr_controller->applyAction();
		}

		//Sleep until endtime of loop intervall
		loop_frequency.sleep();
		//Run ROS Processes
		ros::spinOnce();
	}
	printf("//**********************************************\n");
	printf("// Finishing Fast Cooperative Control Framework \n");
	printf("//**********************************************\n");
}

void Scheduler::simulationloop(double simtime){
	//Index
	unsigned i;
	//Printing loop start screen
	printf("//---------------------------------------------\n");
	printf("// Start SIMULATION Control Loop \n");

	//Starting Agents
	for(int it_controller=0;it_controller<controllerlist_.size();it_controller++){
		ptr_controller=controllerlist_[it_controller];
		ptr_controller->startAgents();
	}

	time_framework_=0;
	//Starting fccf loop
	//--------------------------------------------------------------
	for(counter_fccfloop_=0;time_framework_<simtime;counter_fccfloop_++){
		printf("//---------------------------------------------\n");
		//Print time in framework
		printf("loop: %7i ",counter_fccfloop_);
		printf("time: %7.4f\n",time_framework_);

		//Check if event happend
		bool flag_event_happened=false;
		for(int it_event=0;it_event<eventlist_.size();it_event++){
			if(eventlist_[it_event]->check(time_framework_)){
				flag_event_happened=true;
			}
		}
		//Reinit if event happened
		if(flag_event_happened){
			flag_event_happened=false;
			for(int it_controller=0;it_controller<controllerlist_.size();it_controller++){
				controllerlist_[it_controller]->init();
			}
		}
		//Control loop
		for(int it_controller=0;it_controller<controllerlist_.size();it_controller++){
			ptr_controller=controllerlist_[it_controller];
			ptr_controller->computeAction(time_framework_);
			ptr_controller->applyAction();
			ptr_controller->integrateStateExplicitEuler(time_framework_,time_intervall_);
		}

		time_framework_+=time_intervall_;
	}
	printf("//***********************************************\n");
	printf("// Finishing Fast Cooperative Control Simulation \n");
	printf("//***********************************************\n");
}

