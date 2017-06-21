/**
 * @file    OrientationConstraint_20170227.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Exported with Mathematica Code Generator by Jan Dentler
 */

 #include <OrientationConstraint_20170227.h>

/*******************************************************
 * Constructor                                         * 
 *******************************************************/ 
OrientationConstraint_20170227::OrientationConstraint_20170227(int id):Constraint(id){
    dim_x_=5;
    dim_xdes_=5;
    dim_u_=4;
    dim_udes_=4;
    dim_y_=0;
    dim_ydes_=0;
    dim_p_=0;
    dim_pc_=7;
    dim_l_=1;
    dim_v_=0;
    dim_eq_=0;
    dim_ineq_=0;
    pc_=defvector(dim_pc_);
    pc_init_=defvector(dim_pc_);
}
OrientationConstraint_20170227::OrientationConstraint_20170227(Agent* agent,int id):OrientationConstraint_20170227(id){
    agent_=agent; 
    agent_->addConstraint(this);
}

/******************************************************** 
 * Stage costs                                          * 
 ********************************************************/ 
void OrientationConstraint_20170227::l(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes){
    double o[10];
    o[0] = -x[0];
    o[1] = -x[2];
    o[2] = xdes[2] + o[1];
    o[3] = cos(pc[2]);
    o[4] = x[3]*xdes[0];
    o[5] = -o[4];
    o[6] = x[3]*x[0];
    o[7] = x[4]*xdes[1];
    o[8] = -o[7];
    o[9] = x[4]*x[1];
    o[10] = pc[1] + o[5] + o[6] + o[8] + o[9];
    o[11] = sin(pc[2]);
    out[0] = pc[0]*(xdes[0] + o[0] + pc[5]*pow(x[3]*(-xdes[1] + x[1]) + x[4]*(xdes[0] + o[0]),2.) + pc[4]*pow(pc[3] + o[3]*o[10] + o[2]*o[11],2.) + pc[6]*\
pow(o[2]*o[3] - o[10]*o[11],2.));
}
void OrientationConstraint_20170227::dldx(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes){
    double o[24];
    o[0] = -x[0];
    o[1] = xdes[0] + o[0];
    o[2] = x[4]*o[1];
    o[3] = -xdes[1];
    o[4] = x[1] + o[3];
    o[5] = x[3]*o[4];
    o[6] = o[2] + o[5];
    o[7] = sin(pc[2]);
    o[8] = -x[2];
    o[9] = xdes[2] + o[8];
    o[10] = cos(pc[2]);
    o[11] = o[9]*o[10];
    o[12] = x[3]*xdes[0];
    o[13] = -o[12];
    o[14] = x[3]*x[0];
    o[15] = x[4]*xdes[1];
    o[16] = -o[15];
    o[17] = x[4]*x[1];
    o[18] = pc[1] + o[13] + o[14] + o[16] + o[17];
    o[19] = o[7]*o[18];
    o[20] = -o[19];
    o[21] = o[11] + o[20];
    o[22] = o[10]*o[18];
    o[23] = o[7]*o[9];
    o[25] = pc[3] + o[22] + o[23];
    o[26] = -xdes[0];
    o[27] = x[0] + o[26];
    out[0] = pc[0]*(-1. - 2.*x[4]*pc[5]*o[6] - 2.*x[3]*pc[6]*o[7]*o[21] + 2.*x[3]*pc[4]*o[10]*o[25]);
    out[1] = pc[0]*(2.*x[3]*pc[5]*o[6] - 2.*x[4]*pc[6]*o[7]*o[21] + 2.*x[4]*pc[4]*o[10]*o[25]);
    out[2] = pc[0]*(-2.*pc[6]*o[10]*o[21] - 2.*pc[4]*o[7]*o[25]);
    out[3] = pc[0]*(2.*pc[5]*o[4]*o[6] - 2.*pc[6]*o[7]*o[21]*o[27] + 2.*pc[4]*o[10]*o[25]*o[27]);
    out[4] = pc[0]*(2.*pc[5]*o[1]*o[6] - 2.*pc[6]*o[4]*o[7]*o[21] + 2.*pc[4]*o[4]*o[10]*o[25]);
}
void OrientationConstraint_20170227::dldu(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes){
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
}
