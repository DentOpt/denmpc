/**
 * @file    MathLib.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   General Purpose Library, contains
 * -----------------------------------------------------------------------
 * Some Fundamental Functions for RHC
 * T. Ohtsuka  '97/10/30~'97/10/31 (rhfunc.c)
 * '00/01/27 (rhfuncu.c)
 * -----------------------------------------------------------------------
 * MathLib contains elemental vector operations
 */

#include <MathLib.h>

/*---- print array ----*/
void MathLib::mprint(std::ostream& ss, unsigned nstate, double *a, const char* name)
{
	ss<<name<<"["<<boost::format("%|4|")%nstate<<"]"<<std::endl;

	unsigned	i, j, tmp ;
	ss<<"\t[       ";
	for(j = 0; j < nstate; j++) {
		ss<<boost::format("%|9|")%j<<"  ";
	}
	ss<<std::endl;
	ss<<"\t[0,:]: ";
	for(j = 0; j < nstate; j++) {
		ss<<boost::format("%+8.7f")%a[j]<<",";
	}
	ss<<std::endl;
}

/*---- print matrix ----*/
void MathLib::mprint(std::ostream& ss, unsigned nhor, unsigned nstate, double **a, const char* name)
{
	ss<<name<<"["<<nstate<<","<<nhor<<"]"<<std::endl;
	unsigned	i, j, tmp ;
	ss<<"\t           ";
	for(j = 0; j < nstate; j++) {
		ss<<boost::format("%|9|")%j<<"  ";
	}
	ss<<std::endl;
	for(i = 0; i < nhor; i++){
		ss<<"\t["<<boost::format("%|4|")%i<<",:]: ";
		for(j = 0; j < nstate; j++) {
			ss<<boost::format("%+8.7f")%a[i][j]<<",";
		}
		ss<<std::endl;
	}
}


/*---- print array ----*/
void MathLib::mprint(unsigned nstate, double *a, const char* name)
{
	printf("%s[%ux1]=\n",name,nstate);
	unsigned	i, j, tmp ;
	printf("\t      ");
	for(j = 0; j < nstate; j++) {
		printf("%9u ",j);
	}
	printf("\n");
	printf("\t[0,:]: ");
	for(j = 0; j < nstate; j++) {
		printf("%+8.7f,",a[j]);
	}
	printf("\n");
}

void MathLib::mprint(std::vector<double>a, const char* name)
{
	printf("%s[%lux1]=\n",name,a.size());
	unsigned	i, j, tmp ;
	printf("\t      ");
	for(j = 0; j < a.size(); j++) {
		printf("%9u ",j);
	}
	printf("\n");
	printf("\t[0,:]: ");
	for(j = 0; j < a.size(); j++) {
		printf("%+8.7f,",a[j]);
	}
	printf("\n");
}


/*---- print matrix ----*/
void MathLib::mprint(unsigned nhor, unsigned nstate, double **a, const char* name)
{
	printf("%s[%ux%u]=\n",name,nhor,nstate);
	unsigned	i, j, tmp ;
	printf("\t      ");
	for(j = 0; j < nstate; j++) {
		printf("%9u ",j);
	}
	printf("\n");
	for(i = 0; i < nhor; i++){
		printf("\t[%2u,:]: ",i);
		for(j = 0; j < nstate; j++) {
			printf("%+8.7f,",a[i][j]);
		}
		printf("\n");
	}
}

void MathLib::PoseStamped2State(std::vector<double>& state, const geometry_msgs::PoseStamped msgPose, double laststateupdatetimestamp, double& newstatetimestamp){
	tf::Quaternion qpresent;
	    //Initialize temporary variables
	    double yaw,pitch,roll,dtinput;
	    dtinput=msgPose.header.stamp.toSec()-laststateupdatetimestamp;

		#ifdef DEBUG
	    	printf("PoseStamped2Vector... start \n");
	    	printf("msg timestamp: %f, ",presentpose.header.stamp.toSec());
	    	printf("previous timestamp: %f, ",lastpose.header.stamp.toSec());
	    	printf("time difference: %f\n",dtinput);
		#endif

	    if(dtinput>0){
	        //Approximate UAV Lateral Velocities
	    	state[3]=(msgPose.pose.position.x-state[0])/dtinput;//dx
	    	state[4]=(msgPose.pose.position.y-state[1])/dtinput;//dy
	    	state[5]=(msgPose.pose.position.z-state[2])/dtinput;//dz
	        //Set UAV Position x
	    	state[0]=msgPose.pose.position.x;//x
	    	state[1]=msgPose.pose.position.y;//y
	    	state[2]=msgPose.pose.position.z;//z

	        //Set Euler Angles to phi
	        tf::quaternionMsgToTF(msgPose.pose.orientation, qpresent);
	        //Consider Yaw Pitch Roll sequence within the statevector
	        tf::Matrix3x3(qpresent).getEulerYPR(yaw, pitch, roll);//yaw,pitch,roll

	        //Approximate UAV Lateral Velocities
	        state[9] =atan2(sin(roll -state[6]),cos(roll -state[6]))/dtinput; //droll
	        state[10]=atan2(sin(pitch-state[7]),cos(pitch-state[7]))/dtinput; //dpitch
	        state[11]=atan2(sin(yaw  -state[8]),cos(yaw  -state[8]))/dtinput; //dyaw
	        //Approximate UAV Lateral Velocities
	        state[6]=roll; //roll
	        state[7]=pitch; //pitch
	        state[8]=yaw; //yaw

			#ifdef DEBUG
	        	//Print position and lateral velocity
	        	printf("Vector \n");;
	        	printf("x:	   %+2.5f,",state[0]);
	        	printf("y:	   %+2.5f,",state[1]);
	        	printf("z:	   %+2.5f,",state[2]);
	        	printf("dx:	   %+2.5f,",state[3]);
	        	printf("dy:	   %+2.5f,",state[4]);
	        	printf("dz:	   %+2.5f\n",state[5]);
	        	printf("roll:  %+2.5f,",state[0]);
	        	printf("pitch: %+2.5f,",state[0]);
	        	printf("yaw:   %+2.5f,",state[0]);
	        	printf("droll: %+2.5f,",state[0]);
	        	printf("dpitch:%+2.5f,",state[0]);
	        	printf("dyaw:  %+2.5f\n",state[0]);
			#endif

	    }
	    newstatetimestamp=msgPose.header.stamp.toSec();
}
