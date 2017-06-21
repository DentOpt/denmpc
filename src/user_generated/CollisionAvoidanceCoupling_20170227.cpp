/**
 * @file    CollisionAvoidanceCoupling_20170227.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Exported with Mathematica Code Generator by Jan Dentler
 */

#include "CollisionAvoidanceCoupling_20170227.h"

CollisionAvoidanceCoupling_20170227::CollisionAvoidanceCoupling_20170227(int id){
		//Set initial values
		id_		 =id;
		dim_x1_   =6;
		dim_xdes1_=6;
		dim_u1_   =4;
		dim_udes1_=4;
		dim_y1_   =6;
		dim_ydes1_=0;
		dim_p1_   =8;
		dim_x2_   =6;
		dim_xdes2_=6;
		dim_u2_   =4;
		dim_udes2_=4;
		dim_y2_   =6;
		dim_ydes2_=0;
		dim_p2_   =8;
		dim_pc_   =3;
		dim_l_   =1;
		dim_v_   =0;
		dim_eq_  =0;
		dim_ineq_ =0;
	pc_      =defvector(dim_pc_);

	pc_init_ =defvector(dim_pc_);

}

CollisionAvoidanceCoupling_20170227::CollisionAvoidanceCoupling_20170227(double* pc, int id):CollisionAvoidanceCoupling_20170227(id){
	this->setParameter(pc);
	this->setInitialParameter(pc);
}

CollisionAvoidanceCoupling_20170227::CollisionAvoidanceCoupling_20170227(Agent* agent1, Agent* agent2, double* pc, int id):CollisionAvoidanceCoupling_20170227(id){
		agent1->addCouplingAsAgent1(this);
		agent2->addCouplingAsAgent2(this);
		this->setParameter(pc);
		this->setInitialParameter(pc);
}
void CollisionAvoidanceCoupling_20170227::l(double  *out,double t, double *x1, double *x2, double *u1, double *u2, double *p1, double *p2, double *pc, double *xdes1, double *xdes2, double *udes1, double *udes2){
#ifdef DEBUG_FUNCTION_TRACE
	cout<<"exec l()"<<endl;
#endif
/*pc1*pow(exp(pc2*pow(x1_W,2)-2*pc2*x1_W*x2_W+pc2*pow(x2_W,2)+pc2*pow(y1_W,2)-2*pc2*y1_W*y2_W+pc2*pow(y2_W,2)+pc2*pow(z1_W,2)-2*pc2*z1_W*z2_W+pc2*pow(z2_W,2)-pc2*pc0)+1,-1)*/
	out[0]=pc[1]*pow(exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+1,-1);
}
void CollisionAvoidanceCoupling_20170227::dldx(double  *out1,double  *out2,double t, double *x1, double *x2, double *u1, double *u2, double *p1, double *p2, double *pc, double *xdes1, double *xdes2, double *udes1, double *udes2){
#ifdef DEBUG_FUNCTION_TRACE
	cout<<"exec dldx()"<<endl;
#endif
/*
[2*pc1*pow(exp(pc2*pow(x1_W,2)-2*pc2*x1_W*x2_W+pc2*pow(x2_W,2)+pc2*pow(y1_W,2)-2*pc2*y1_W*y2_W+pc2*pow(y2_W,2)+pc2*pow(z1_W,2)-2*pc2*z1_W*z2_W+pc2*pow(z2_W,2)-pc2*pc0)+1,-2)*pc2*x1_W*exp(pc2*pow(x1_W,2)-2*pc2*x1_W*x2_W+pc2*pow(x2_W,2)+pc2*pow(y1_W,2)-2*pc2*y1_W*y2_W+pc2*pow(y2_W,2)+pc2*pow(z1_W,2)-2*pc2*z1_W*z2_W+pc2*pow(z2_W,2)-pc2*pc0)-2*pc1*pow(exp(pc2*pow(x1_W,2)-2*pc2*x1_W*x2_W+pc2*pow(x2_W,2)+pc2*pow(y1_W,2)-2*pc2*y1_W*y2_W+pc2*pow(y2_W,2)+pc2*pow(z1_W,2)-2*pc2*z1_W*z2_W+pc2*pow(z2_W,2)-pc2*pc0)+1,-2)*pc2*x2_W*exp(pc2*pow(x1_W,2)-2*pc2*x1_W*x2_W+pc2*pow(x2_W,2)+pc2*pow(y1_W,2)-2*pc2*y1_W*y2_W+pc2*pow(y2_W,2)+pc2*pow(z1_W,2)-2*pc2*z1_W*z2_W+pc2*pow(z2_W,2)-pc2*pc0) 2*pc1*pow(exp(pc2*pow(x1_W,2)-2*pc2*x1_W*x2_W+pc2*pow(x2_W,2)+pc2*pow(y1_W,2)-2*pc2*y1_W*y2_W+pc2*pow(y2_W,2)+pc2*pow(z1_W,2)-2*pc2*z1_W*z2_W+pc2*pow(z2_W,2)-pc2*pc0)+1,-2)*pc2*y1_W*exp(pc2*pow(x1_W,2)-2*pc2*x1_W*x2_W+pc2*pow(x2_W,2)+pc2*pow(y1_W,2)-2*pc2*y1_W*y2_W+pc2*pow(y2_W,2)+pc2*pow(z1_W,2)-2*pc2*z1_W*z2_W+pc2*pow(z2_W,2)-pc2*pc0)-2*pc1*pow(exp(pc2*pow(x1_W,2)-2*pc2*x1_W*x2_W+pc2*pow(x2_W,2)+pc2*pow(y1_W,2)-2*pc2*y1_W*y2_W+pc2*pow(y2_W,2)+pc2*pow(z1_W,2)-2*pc2*z1_W*z2_W+pc2*pow(z2_W,2)-pc2*pc0)+1,-2)*pc2*y2_W*exp(pc2*pow(x1_W,2)-2*pc2*x1_W*x2_W+pc2*pow(x2_W,2)+pc2*pow(y1_W,2)-2*pc2*y1_W*y2_W+pc2*pow(y2_W,2)+pc2*pow(z1_W,2)-2*pc2*z1_W*z2_W+pc2*pow(z2_W,2)-pc2*pc0) 2*pc1*pow(exp(pc2*pow(x1_W,2)-2*pc2*x1_W*x2_W+pc2*pow(x2_W,2)+pc2*pow(y1_W,2)-2*pc2*y1_W*y2_W+pc2*pow(y2_W,2)+pc2*pow(z1_W,2)-2*pc2*z1_W*z2_W+pc2*pow(z2_W,2)-pc2*pc0)+1,-2)*pc2*z1_W*exp(pc2*pow(x1_W,2)-2*pc2*x1_W*x2_W+pc2*pow(x2_W,2)+pc2*pow(y1_W,2)-2*pc2*y1_W*y2_W+pc2*pow(y2_W,2)+pc2*pow(z1_W,2)-2*pc2*z1_W*z2_W+pc2*pow(z2_W,2)-pc2*pc0)-2*pc1*pow(exp(pc2*pow(x1_W,2)-2*pc2*x1_W*x2_W+pc2*pow(x2_W,2)+pc2*pow(y1_W,2)-2*pc2*y1_W*y2_W+pc2*pow(y2_W,2)+pc2*pow(z1_W,2)-2*pc2*z1_W*z2_W+pc2*pow(z2_W,2)-pc2*pc0)+1,-2)*pc2*z2_W*exp(pc2*pow(x1_W,2)-2*pc2*x1_W*x2_W+pc2*pow(x2_W,2)+pc2*pow(y1_W,2)-2*pc2*y1_W*y2_W+pc2*pow(y2_W,2)+pc2*pow(z1_W,2)-2*pc2*z1_W*z2_W+pc2*pow(z2_W,2)-pc2*pc0)                                                                                                                                                                                                                                                                                                                                                   0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     0                                                                                                                                                                                                                                                                                                                                                  ]
*/
	out1[0]=-2*pc[1]*pow(exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+1,-2)*pc[2]*x1[0]*exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+2*pc[1]*pow(exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+1,-2)*pc[2]*x2[0]*exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0]);
	out1[1]=-2*pc[1]*pow(exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+1,-2)*pc[2]*x1[1]*exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+2*pc[1]*pow(exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+1,-2)*pc[2]*x2[1]*exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0]);
	out1[2]=-2*pc[1]*pow(exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+1,-2)*pc[2]*x1[2]*exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+2*pc[1]*pow(exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+1,-2)*pc[2]*x2[2]*exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0]);
	out1[3]=0;
	out1[4]=0;
	out1[5]=0;
	out2[0]=2*pc[1]*pow(exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+1,-2)*pc[2]*x1[0]*exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])-2*pc[1]*pow(exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+1,-2)*pc[2]*x2[0]*exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0]);
	out2[1]=2*pc[1]*pow(exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+1,-2)*pc[2]*x1[1]*exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])-2*pc[1]*pow(exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+1,-2)*pc[2]*x2[1]*exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0]);
	out2[2]=2*pc[1]*pow(exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+1,-2)*pc[2]*x1[2]*exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])-2*pc[1]*pow(exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0])+1,-2)*pc[2]*x2[2]*exp(pc[2]*pow(x1[0],2)-2*pc[2]*x1[0]*x2[0]+pc[2]*pow(x2[0],2)+pc[2]*pow(x1[1],2)-2*pc[2]*x1[1]*x2[1]+pc[2]*pow(x2[1],2)+pc[2]*pow(x1[2],2)-2*pc[2]*x1[2]*x2[2]+pc[2]*pow(x2[2],2)-pc[2]*pc[0]);
	out2[3]=0;
	out2[4]=0;
	out2[5]=0;
}
void CollisionAvoidanceCoupling_20170227::dldu(double  *out1,double  *out2,double t, double *x1, double *x2, double *u1, double *u2, double *p1, double *p2, double *pc, double *xdes1, double *xdes2, double *udes1, double *udes2){
#ifdef DEBUG_FUNCTION_TRACE
	cout<<"exec dldu()"<<endl;
#endif
/*
[0 0 0 0]
*/
	out1[0]=0;
	out1[1]=0;
	out1[2]=0;
	out1[3]=0;
	out2[0]=0;
	out2[1]=0;
	out2[2]=0;
	out2[3]=0;
}
