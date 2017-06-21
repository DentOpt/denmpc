/**
 * @file    Indexing.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Index Container
 *
 * Indexing represents the container class for all indices
 * within the concatenated optimization variables vectors.
 */

#ifndef _INDEXING_H
#define _INDEXING_H

struct AgentIndex{
	//Pointers
	int index_x_;	 //Index of state
	int index_u_;	 //Index of control
	int index_xdes_;	 //Index of desired state
	int index_udes_;	 //Index of desired control
	int index_d_;	 //Index of disturbance
	int index_p_;	 //Index of parameters
	int index_lambda_;//index of state lagrange multiplier

	void initIndex(int _index_x,int _index_u,int _index_xdes,int _index_udes,int _index_d,int _index_p,int _index_lambda){
		index_x_   	 =_index_x;
		index_u_   	 =_index_u;
		index_xdes_	 =_index_xdes;
		index_udes_	 =_index_udes;
		index_d_	 =_index_d;
		index_p_	 =_index_p;
		index_lambda_=_index_lambda;
	}
};

//struct AgentIndex{
//	//Pointers
//	int index_x;	 //Index of state
//	int index_u;	 //Index of control
//	int index_xdes;	 //Index of desired state
//	int index_udes;	 //Index of desired control
//	int index_p;	 //Index of parameters
//	int index_lambda;//index of state lagrange multiplier
//	int index_mu;	 //index of equality lagrange multiplier
//	int index_mui;	 //index of inequality lagrange multiplier
//
//	void initIndex(int _index_x,int _index_u,int _index_xdes,int _index_udes,int _index_p,int _index_lambda,int _index_mu,int _index_mui){
//		index_x   	=_index_x;
//		index_u   	=_index_u;
//		index_xdes	=_index_xdes;
//		index_udes	=_index_udes;
//		index_p	  	=_index_p;
//		index_lambda=_index_lambda;
//	}
//};

struct ConstraintIndex{
	//Pointers
	int index_x_;	 //Index of state
	int index_u_;	 //Index of control
	int index_xdes_;	 //Index of desired state
	int index_udes_;	 //Index of desired control
	int index_p_;	 //Index of parameters
	int index_pc_;	 //Index of parameters
	int index_mu_;	 //index of equality lagrange multiplier
	int index_mui_;	 //index of inequality lagrange multiplier

	void initIndex(int _index_x,int _index_u,int _index_xdes,int _index_udes,int _index_p,int _index_pc,int _index_mu,int _index_mui){
		index_x_   	=_index_x;
		index_u_   	=_index_u;
		index_xdes_	=_index_xdes;
		index_udes_	=_index_udes;
		index_p_	=_index_p;
		index_pc_	=_index_pc;
		index_mu_   =_index_mu;
		index_mui_  =_index_mui;
	}
};


struct CouplingIndex{
	//Pointers
	int index_x1_;	 //Index of state
	int index_u1_;	 //Index of control
	int index_xdes1_;	 //Index of desired state
	int index_udes1_;	 //Index of desired control
	int index_p1_;	 //Index of parameters
	int index_x2_;	 //Index of state
	int index_u2_;	 //Index of control
	int index_xdes2_;	 //Index of desired state
	int index_udes2_;	 //Index of desired control
	int index_p2_;	 //Index of parameters
	int index_pc_;	 //Index of parameters
	int index_mu_;	 //index of equality lagrange multiplier
	int index_mui_;	 //index of inequality lagrange multiplier

	void initIndex(
			int _index_x1,   int _index_x2,
			int _index_u1, 	 int _index_u2,
			int _index_xdes1,int _index_xdes2,
			int _index_udes1,int _index_udes2,
			int _index_p1,	 int _index_p2,int _index_pc,
			int _index_mu,   int _index_mui){
		index_x1_   =_index_x1;
		index_u1_   =_index_u1;
		index_xdes1_=_index_xdes1;
		index_udes1_=_index_udes1;
		index_p1_	=_index_p1;

		index_x2_   =_index_x2;
		index_u2_   =_index_u2;
		index_xdes2_=_index_xdes2;
		index_udes2_=_index_udes2;
		index_p2_	=_index_p2;
		index_pc_	=_index_pc;

		index_mu_   =_index_mu;
		index_mui_  =_index_mui;
	}
};

#endif
