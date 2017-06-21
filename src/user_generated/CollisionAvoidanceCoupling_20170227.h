/**
 * @file    CollisionAvoidanceCoupling_20170227.h
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Exported with Mathematica Code Generator by Jan Dentler
 */

#ifndef COLLISIONAVOIDANCECOUPLING_H_
#define COLLISIONAVOIDANCECOUPLING_H_

#include "Coupling.h"
#include "MathLib.h"
using namespace MathLib;

class CollisionAvoidanceCoupling_20170227:public Coupling{
public:
	CollisionAvoidanceCoupling_20170227(int id=0);
	CollisionAvoidanceCoupling_20170227(double* pc, int id=0);
	CollisionAvoidanceCoupling_20170227(Agent* agent1, Agent* agent2, double* pc, int id=0);
void l(double  *out,double t, double *x1, double *x2, double *u1, double *u2, double *p1, double *p2, double *pc, double *xdes1, double *xdes2, double *udes1, double *udes2);
void dldx(double  *out1,double  *out2,double t, double *x1, double *x2, double *u1, double *u2, double *p1, double *p2, double *pc, double *xdes1, double *xdes2, double *udes1, double *udes2);
void dldu(double  *out1,double  *out2,double t, double *x1, double *x2, double *u1, double *u2, double *p1, double *p2, double *pc, double *xdes1, double *xdes2, double *udes1, double *udes2);
};
#endif
