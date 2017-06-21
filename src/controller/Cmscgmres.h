/**
 * @file    Cmscgmres.h
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Condensed Multiple Shooting Generalized Minimal Residuum Method (CMSCGMRES)
 *
 *	This is a modification of the CMSCGMRES developed by
 *  Ohtsuka, T., “A Continuation/GMRES Method for Fast Computation of Nonlinear Receding Horizon Control,” Automatica, Vol. 40, No. 4, Apr. 2004, pp. 563-574.
 *  Seguchi, H., and Ohtsuka, T., “Nonlinear Receding Horizon Control of an Underactuated Hovercraft,” International Journal of Robust and Nonlinear Control, Vol. 13, Nos. 3-4, Mar.-Apr. 2003, pp. 381-398.
 *
 *  The CMSCGMRES has been extended by an orthogonalization within the GMRES Solver which allows
 *  suboptimal solutions of infeasible problems
 *  and a security measures for infeasible cases.
 */

#ifndef CMSCGMRES_H_
#define CMSCGMRES_H_

#include "Controller.h"

	//=================================================//
	// Preprocessor Commands to turn of / on Debugging //
	//=================================================//

//#define TRACE_ON
//#define RESET_DU
//#define RESET_DX
//#define DEBUG_CGMRES
//#define CONTROLLER_INFO
//#define CGMRES_INIT_INFO
//#define STATE_INFO
//#define DEBUG_CGMRES

class Cmscgmres : public Controller{
protected:
	/*-------------- Global Variables Defined by User -------------- */
	std::string type_;

	//========================================//
	// Controller Parameters   				  //
	//========================================//
	double 		alpha_;
	double 		zeta_;
	double 		rtol_;
	double 		hdir_;
	double 		htau_;
	int  		kmax_;
	int 		nhor_;

	//===============================================//
	// Controller Global Variables / Pointers / Flags//
	//===============================================//
	double 	tsim0_;
	int  	controlcounter_;
	int	   	dim_f_, dim_g_, dim_x_,dim_p_;
	double 	onemhdir_, hdirbht_, htbhdir_, onemzetahdir_, onemzetahdir2_;
	double 	sum_;
	double 	t_, ts_;
	double 	tauf_;
	double 	*tau_;
	double 	**U_, **X_, **Uk_, **Fk_, **Hk_, **U1_;
	double 	**F_, **G_, **F1s_, **G1s_, **H1s_, **F1_, **F2_, **G1_, **H1_;
	double 	/* *x,*/ *x1s_, *bvec_, *duvec_, *dxvec_, *dutmp_, *errvec_;
	double 	*lmd0_, *hu0_;
	double 	thor_;
	double 	ht_;
	double* x0_;
	double* u0_;
	double* udes_;
	double* p_;
	double* x1_;
	double** f3_;
	//Function Pointers
	typedef  void (Cmscgmres::*funcptr_cgmresfunc)(unsigned, double *, double *);
	typedef  void (Cmscgmres::*funcPtr_func)      (double,double*,double*,double*);

	//========================================//
	// CMSCGMRES SYSTEM FUNCTIONS INTERFACE   //
	//========================================//

	/**-------------- hufunc --------------
	 * contains dHd[u,a_slack]
	 *------------------------------------
	 * @param  time
	 * @param  states
	 * @param  adjoint states
	 * @param  inputs
	 * @param  dHdu \huhu
	 * @return void
	 */
	inline void hufunc(double t, double* x, double* lambda, double* optvar, double* dHduout){
		//		double* u=optvar;
		//		double* mu=optvar+u_conc_dim;
		dHdu(dHduout,t,x,optvar,d_conc_,p_conc_,xdes_conc_,udes_conc_,lambda);
	};

	/**---- lpfunc ----
	 * 	 contains -dHdx with linp=[x,optvar]
	 *------------------------------------
	 * @param time
	 * @param adjoint states
	 * @param [states,inputs,desiredstates]
	 * @param derivate of adjointstates
	 * @return void
	 */
	inline void lpfunc(double t, double* lambda, double* linp, double* lambdaprime){
//		//linp=[x,optvar] //Primal Barrier => linp=[x,optvar]
//		double* x=linp;
//		double* optvar=linp +dim_x_conc_;
		minusdHdx(lambdaprime,t,linp,linp+dim_x_conc_,d_conc_,p_conc_,xdes_conc_,udes_conc_,lambda);
	};

	/**---- dxpfunc= System ODE ----
	 * @param time
	 * @param states
	 * @param inputs
	 * @param derivate of states
	 * @return void
	 */
	inline void xpfunc(double t, double* x, double* u, double* dfdx){
		//this->function_fsys_ohtsuka(dfdx,t,x,u,timevarpar);
		f(dfdx,t,x,u,d_conc_,p_conc_);
	};

	/**---- Phix = Final costs function ----
	 * @param time
	 * @param states
	 * @param endcosts
	 * @return void
	 */
	inline void phix(double t, double* x, double* out){
		//this->function_dvdx(phx1,x,xdes);
		//v(out,t,x,p,xdes);
		dvdx(out,t,x,p_conc_,xdes_conc_);
	};


	//========================================//
	//  MAIN CMSCGMRES ROUTINES 			  //
	//========================================//

	/*---- k Givens Roations used in GMRES ----
	 * rotate with c, s so that [[s,c];[-s c]]*[a;b]=[r;0]
	 * @param  k : number of rows/columns
	 * @param  c : Spur Elements
	 * @param  s : Digonal Elements
	 * @param  v : [r,0];
	 * @return void
	 */
	inline void givrot(unsigned k, double *c, double *s, double *v);

	/**--------------------------------------------------------
	 * Linear Equation Subroutine
	 * GMRES (Generalized Minimum Residual)
	 * Cf. C.T.Kelley: Iterative Methods for Linear and Nonlinear Equations
	 * T.Ohtsuka  '00/01/24
	 * @param  axfunc(unsigned n, double *x, double *ax): a function that gives A*x
	 * @param  n: dim x
	 * @param  b: the right-hand-side of the equation, A*x = b
	 * @param  x: initial guess, to be replaced with the solution
	 * @param  kmax: number of iteration
	 * @param  err: residual norms, |b - A*x_i| (i=1,...,n+1)
	 * @return void
	 */
	inline void nfgmres(funcptr_cgmresfunc axfunc, unsigned n, double *b, double *x, unsigned kmax, double *err);

	/**--------------------------------------------------------
	 * Simultaneous Ordinaly Diferential Equation Subroutine
	 * Runge-Kutta_Gill Method	(Original by T.Murayama)
	 *
	 * ---- Variation of rkg() : dx/dt is also returned. ----
	 * ----  (for Start of the Adams method)  ----
	 * ----------------------------------------------------------
	 * @param  void (*func)()
	 * @param time
	 * @param states
	 * @param	inputs
	 * @param stepsize
	 * @param state dimension
	 * @param ans
	 * @param fxy
	 * @return void
	 */
	inline void nfrkginpex(funcPtr_func func,double x,double* y,double* u,double h,unsigned dim,double* ans,double* fxy);

	/**--------------------------------------------------------
	 * Simultaneous Ordinaly Diferential Equation Subroutine
	 * Adams Method (Predictor-Corrector Method)
	 * Predictor: Adams-Bashforth
	 * Corrector: Adams-Moulton
	 * T.Ohtsuka  '92/10/24
	 */
	inline void nfadamsinp(funcPtr_func func,double x,double* y,double* u,double* f1,double* f2,double* f3,double h,unsigned dim,double* ans);

	/**--------------------------------------------------------
	 * Simultaneous Ordinary Differential Equation Subroutine	Euler Method (Forward Difference)
	 * ans[i] = y[i] + h * func(x,y,u,fval);
	 * ----------------------------------------------------------
	 * @param evaluation function
	 * @param time[k]
	 * @param states[k]
	 * @param inputs[k]
	 * @param stepsize
	 * @param state dimension
	 * @param states[k+1]
	 * @return void
	 */
	inline void nfeulerinp(funcPtr_func func, double x, double* y, double* u, double h, unsigned dim, double* ans);

	/**--------Initial conditions for dHdudt ---------
	 * @param time
	 * @param dudt0
	 * @param dHdudt
	 * @return void
	 */
	inline void dhu0func(unsigned dimu, double *du0, double *dhu);

	/**--------Compute u, that fulfills optimality at initial time tau0=t ---------
	 * @param states
	 * @param inputs
	 * @return void
	 */
	inline void getOptimalInitialValues(double* x, double* u);

	/**-------------- dHdu of present horizon ---------
	 * @param time
	 * @param states
	 * @param inputs
	 * @param dHdu(t,x,u)
	 * @return void
	 */
	inline void errfunc(double t, double *x, double **u, double **hu);

	/**-------------- unew --------------
	 * calculate lefthand side:
	 * DhF(U; x; t : dU; dx; 1)
	 *----------------------------------
	 * @param dimension of u
	 * @param derivate of inputs
	 * @param lefthand side
	 * @return void
	 */
	inline void adufunc(unsigned n, double *du, double *adu);

	/**-------------- unew --------------
	 * calculate and save new controller actuation in u
	 *----------------------------------
	 * @param time
	 * @param states
	 * @param file for saving error
	 * @param file for saving costs
	 * @return void
	 */
	inline void unew(double t, double* x, double* x1, double* u);

	/**-------------- Ffunc --------------
	 * calculate F(U,X,x,t): optimization variable derivative
	 *----------------------------------
	 * @param **Uk
	 * @param **Xk
	 * @param *xk
	 * @param tk
	 * @param **Fk
	 * @return void
	 */
	inline void Ffunc(double **Uk, double **Xk, double *xk, double tk, double **Fk);

	/**-------------- Gfunc --------------
	 * calculate G(U,X,x,t): state derivative
	 *----------------------------------
	 * @param **Uk
	 * @param **Xk
	 * @param *xk
	 * @param tk
	 * @param **Fk
	 * @return void
	 */
	inline void Gfunc(double **Uk, double **Xk, double *xk, double tk, double **Gk);

	/**-------------- Hfunc --------------
	 * calculate H(U,X,x,t)
	 *----------------------------------
	 * @param **Uk
	 * @param **Zk
	 * @param *xk
	 * @param tk
	 * @param **Hk
	 * @return void
	 */
	inline void Hfunc(double **Uk, double **Zk, double *xk, double tk, double **Hk);


	//========================================//
	// Internal initialization routines		  //
	//========================================//

	/**-------------- initVectors --------------
	 * sets arrays with current values
	 *------------------------------------
	 * @return void
	 */
	inline void initVectors();

	/**-------------- allocateMemory --------------
	 * allocates Memory for the controller
	 *------------------------------------
	 * @return void
	 */
	void allocateMemory();

	/**-------------- freeMemory --------------
	 * frees Memory for the controller
	 *------------------------------------
	 * @return void
	 */
	void freeMemory();


public:
	//========================================//
	//  MAIN ACCES METHODS					  //
	//========================================//

	/**-------------- Constructor --------------
	 * @param _listofagents
	 * @param _kmax
	 * @param _thor
	 * @param _ht
	 * @param _nhor
	 * @param _rtol
	 * @param _zeta
	 * @param _alpha
	 * @param _hdir
	 * @return void
	 */
	Cmscgmres(std::vector<Agent*> _listofagents,int _id,
			int _kmax=6,double _thor=1, double _ht=0.1,
			int _nhor=10,double _rtol= 1e-8,double _zeta=10,
			double _alpha=2,double _hdir=0.001);

	/**Destructor	*/
	~Cmscgmres();

	/**-------------- Compute controller action --------------
	 * @param _time
	 * @return void
	 */
	void computeAction(double _time);

	/**-------------- Init controller --------------
	 * 	allocate memory and initialize controls
	 * 	according to the optimality conditions
	 *----------------------------------
	 * @param _time
	 * @return void
	 */
	void init();


	//========================================//
	//  SETTER METHODS FOR SOLVER PARAMETERS  //
	//========================================//

	/**-------------- set Solver Tolerance --------------
	 * @param _tol
	 * @return void
	 */
	void setTolerance(double _tol){
		rtol_=_tol;
		init();
	}

	/**-------------- set Horizon Expansion Factor --------------
	 * @param _alpha
	 * @return void
	 */
	void setHorizonExpansionFactor(double _alpha){
		alpha_=_alpha;
		init();
	}

	/**-------------- set Maximum number of CGMRES iterations --------------
	 * @param _kmax
	 * @return void
	 */
	void setMaximumNumberofIterations(double _kmax){
		kmax_=_kmax;
		init();
	}

	/**-------------- set Prediction Horizon length --------------
	 * @param _thor
	 * @return void
	 */
	void setHorizonLength(double _thor){
		thor_=_thor;
		init();
	}

	/**-------------- set Number of Diskretisation points of the horizon --------------
	 * @param _nhor
	 * @return void
	 */
	void setHorizonDiskretization(double _nhor){
		nhor_=_nhor;
		init();
	}

	/**-------------- set the discretisation step of the hessian approximation --------------
	 * @param _hdir
	 * @return void
	 */
	void setForwardDifferenceStep(double _hdir){
		hdir_=_hdir;
		onemhdir_ 	= 1 - hdir_ / ht_;
		hdirbht_ 	= hdir_ / ht_;
		htbhdir_ 	= ht_ / hdir_;
		onemzetahdir_= 1 - zeta_*hdir_;
		init();
	}

	/**-------------- set the update interval, with which the CMSCGMRES is calculating --------------
	 * @param _updateintervall
	 * @return void
	 */
	void setUpdateIntervall(double _updateintervall){
		ht_=_updateintervall;
		onemhdir_ 	= 1 - hdir_ / ht_;
		hdirbht_ 	= hdir_ / ht_;
		htbhdir_ 	= ht_ / hdir_;
		onemzetahdir_= 1 - zeta_*hdir_;
		init();
	}

	/**-------------- set the desired solver convergence factor of the dynamics --------------
	 * @param _zeta
	 * @return void
	 */
	void setContiunationConvergenceFactor(double _zeta){
		zeta_=_zeta;
		onemzetahdir_= 1 - zeta_*hdir_;
		init();
	}


	//=======================//
	//  VECTOR MANIPULATION  //
	//=======================//
	/**---- a[m][n] -> b[m][n] ----*/
	void mmov( int n, double *a, double *b );
	/**---- a[m][n] -> b[m][n] ----*/
	void mmov( int m, int n, double *a, double *b );
	/**---- a[m][n] + b[m][n] -> c[m][n] ----*/
	void madd( int m, int n, double *a, double *b, double *c );
	/**---- a[m][n] - b[m][n] -> c[m][n] ----*/
	void msub( int m, int n, double *a, double *b, double *c );
	/**---- k * a[m][n] -> b[m][n] ----*/
	void mmulsc( int m, int n, double *a, double k, double *b );
	/**---- a[m][n] -> -a[m][n] ----*/
	void mminus( int m, int n, double *a, double *b );
	/**---- a[m][n] / k -> b[m][n] ----*/
	void mdivsc( int m, int n, double *a, double k, double *b );
	/**---- Inner Product of a[m] and b[m] ----*/
	double	mvinner( int m, double *a, double *b );
	/**---- Define an n-Dimensional Vector ----*/
	double *defvector(int n);
	/**---- UnDefine an n-Dimensional Vector ----*/
	void freevector(double *v);
	/**---- Define an n by m Matrix ----*/
	double **defmatrix(int n, int m);
	/**---- Undefine an n by m Matrix ----*/
	void freematrix(double **a);

};





#endif
