/**
 * @file    OrientationConstraint_20170227.h
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Exported with Mathematica Code Generator by Jan Dentler
 */


#ifndef ORIENTATIONCONSTRAINT_20170227_H_
#define ORIENTATIONCONSTRAINT_20170227_H_

#include <Constraint.h>

/*******************************************************
 * Constructor                                         * 
 *******************************************************/ 

class OrientationConstraint_20170227:public Constraint{
public:
OrientationConstraint_20170227(int id=0);
OrientationConstraint_20170227(Agent* agent,int id=0);
void l(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes);
void dldx(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes);
void dldu(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes);
};
#endif
