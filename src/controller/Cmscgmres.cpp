/**
 * @file    Cmscgmres.cpp
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

#include "Cmscgmres.h"

//Cmscgmres::Cmscgmres(
//		int _id, /**< id_ of controlled Agent*/
//		Agent* _agent, /**< Controlled Agent*/
//		double _updateintervall, /**< update step -> discretization of global time*/
//		double _calctimemax, /**< Maximal calculation time at disposal*/
//		double _initialtimestamp, /**< Initial timestamp of framework;*/
//		double _thor,				/**< horizon length*/
//		int    _nhor,					/**< horizon discretization step*/
//		double _hdir,    				/**< Forward approximation step*/
//		double _alpha,	/**< Parameter for Variable Horizon, T = thor*(1-exp(-alpha_*t)) */
//		double _zeta,	/**< Parameter for Stabilization of Continuation Method*/
//		double _rtol,	/**< Tolerance of Error in Initial Control Input and Multipliers, u0 */
//		double _kmax 	/**< Number of Iteration in GMRES*/
//) :
Cmscgmres::Cmscgmres(std::vector<Agent*> _listofagents, int _id,
		int _kmax,double _thor, double _ht,
		int _nhor,double _rtol,double _zeta,
		double _alpha,double _hdir
) : Controller(_listofagents, _id){

	this->type_ = "CMSCGMRES Controller";
	this->id_   = _id;
	printf("Controller %i", this->id_);
	printf(" %s Constructor called\n", type_.c_str());
	//Initialize flags
	flag_memoryalreadyallocated_=false;

	//Constants
	nhor_		= _nhor; /*standard =6*/
	alpha_ 		= _alpha;/*standard =2*/
	zeta_  		= _zeta; /*standard =10*/
	rtol_  		= _rtol; /*standard =1e-1*/
	kmax_  		= _kmax; /*standard =6*/
	hdir_		= _hdir; /*standard =0.001*/
	ht_			= _ht;   /*standard =0.1*/
	thor_		= _thor; /*standard =1.0*/
	controlcounter_=0;
	sum_   		= 0;
	onemhdir_ 	= 1 - hdir_ / ht_;
	hdirbht_ 	= hdir_ / ht_;
	htbhdir_ 	= ht_ / hdir_;
	onemzetahdir_= 1 - zeta_*hdir_;

	init();
	//Set flag true, as memory is allocated
	flag_memoryalreadyallocated_=true;

}

//Init the variables
void Cmscgmres::allocateMemory(){

	/*-------------- Dimensions -------------- */
	dim_f_  	= dim_optvar_conc_*nhor_;
	dim_g_ 	= 2*dim_x_conc_*nhor_;

	/* Arrays */
	tau_   	= defvector(nhor_+1);
	U_     	= defmatrix(nhor_,  (int)dim_optvar_conc_);
	X_     	= defmatrix(2*nhor_,(int)dim_x_conc_);
	F_     	= defmatrix(nhor_,  (int)dim_optvar_conc_);
	G_     	= defmatrix(2*nhor_,(int)dim_x_conc_);
	F1s_   	= defmatrix(nhor_,  (int)dim_optvar_conc_);
	G1s_   	= defmatrix(2*nhor_,(int)dim_x_conc_);
	F1_    	= defmatrix(nhor_,  (int)dim_optvar_conc_);
	G1_    	= defmatrix(2*nhor_,(int)dim_x_conc_);
	F2_    	= defmatrix(nhor_,  (int)dim_optvar_conc_);
	H1_    	= defmatrix(2*nhor_,(int)dim_x_conc_);
	U1_    	= defmatrix(nhor_,  (int)dim_optvar_conc_);
	Uk_    	= defmatrix(nhor_,  (int)dim_optvar_conc_);
	Fk_    	= defmatrix(nhor_,  (int)dim_optvar_conc_);
	Hk_    	= defmatrix(2*nhor_,(int)dim_x_conc_);
	bvec_  	= defvector(dim_f_);
	duvec_ 	= defvector(dim_f_);
	dxvec_ 	= defvector(dim_g_);
	dutmp_ 	= defvector(dim_f_);
	errvec_	= defvector(kmax_+1);
	x1_ 	  	= defvector(dim_x_conc_);
	x1s_   	= defvector(dim_x_conc_);
	f3_ 	 	= defmatrix(3,dim_x_conc_);
}

void Cmscgmres::initVectors(){
	//Set x0, u0, f3
	x0_  = x_conc_;
	u0_	 = optvar_conc_;
	p_	 = p_conc_;
	mmov(1,dim_x_conc_,x0_,f3_[0]);
	mmov(1,dim_x_conc_,x0_,f3_[1]);
	mmov(1,dim_x_conc_,x0_,f3_[2]);
}

void Cmscgmres::init(){

	if(flag_memoryalreadyallocated_){
		//Free Memory
		freeMemory();
	}
	//Concatenate
	initConcatenation();
	//Allocate Memory
	allocateMemory();

	//Print Cmscgmres
	//--------------------------------------------------
#ifdef CGMRES_INIT_INFO
	printf("Cmscgmres Parameter|");
	printf(" alpha_:%f,", alpha_);
	printf(" zeta:%f,", zeta_);
	printf(" rtol:%f,", rtol_);
	printf(" kmax:%u,", kmax_);
	printf(" thor:%f,", thor_);
	printf(" nhor_:%u,", nhor_);
	printf(" hdir:%f,", hdir_);
	printf("Cmscgmres VectorDim|");
	printf(" concatenated states:%u,", 	dim_x_conc_);
	printf(" concatenated outputs:%u,", u_conc_dim);
	printf(" concatenated equality constraints:%u,", 	eqcon_conc_dim);
	printf(" concatenated inequality constraints:%u,", 	ineqcon_conc_dim);
	printf(" concatenated parameter vector:%u\n", 		p_conc_dim);
	printf(" optvar_conc_dim_:%u,", 		dim_optvar_conc_);
	printf(" x_conc_dim:%u,", 		dim_x_conc_);
	printf(" dimf:%u,", 		dim_f_);
	printf(" dimg:%u\n", 		dim_g_);
	//Init Cmscgmres
	MathLib::mprint(log_stringstream_,dim_optvar_conc_, u0, "u0");
	MathLib::mprint(log_stringstream_,dim_x_conc_, 	 x0, "x0");
	MathLib::mprint(log_stringstream_,xdes_conc_dim,   xdes_conc, "xdes");
	MathLib::mprint(log_stringstream_,udes_conc_dim,   udes_conc, "udes");
#endif
	getOptimalInitialValues(x_conc_,u_conc_);
#ifdef CGMRES_INIT_INFO
	printf(" AFTER MSCGMRES initialization phase\n");
	MathLib::mprint(log_stringstream_,dim_optvar_conc_, u_conc, "u_conc");
	MathLib::mprint(log_stringstream_,dim_x_conc_, 	 x_conc, "x_conc");
	MathLib::mprint(log_stringstream_,xdes_conc_dim,   xdes_conc, "xdes");
	MathLib::mprint(log_stringstream_,udes_conc_dim,   udes_conc, "udes");
#endif

}



// Destructor
Cmscgmres::~Cmscgmres() {
	freeMemory();
}

void Cmscgmres::freeMemory(){
	/* Arrays */
	free(tau_);
	free(U_);
	free(X_);
	free(F_);
	free(G_);
	free(F1s_);
	free(G1s_);
	free(F1_);
	free(G1_);
	free(F2_);
	free(H1_);
	free(U1_);
	free(Uk_);
	free(Fk_);
	free(Hk_);
	free(bvec_);
	free(duvec_);
	free(dxvec_);
	free(dutmp_);
	free(errvec_);
	free(x1_);
	free(x1s_);
	free(f3_);

}

//void Cmscgmres::calcControlUpdate(double globaltime,double* presentpose,double* desiredpose)
void Cmscgmres::computeAction(double t) {
	int i;
	ts_ = t + ht_;
	//Get Time
	double clock(ros::Time::now().toSec());

	//Predict next state by integration
	if(controlcounter_<3){
		nfrkginpex(&Cmscgmres::xpfunc,t,x_conc_,optvar_conc_,ht_,dim_x_conc_, x1_,f3_[controlcounter_]);
	}
	else{
		nfadamsinp(&Cmscgmres::xpfunc,t,x_conc_,optvar_conc_,f3_[2],f3_[1],f3_[0],ht_,dim_x_conc_, x1_);
	}

	if(flag_show_controllerstates_){
		log_stringstream_<<"Before Loop"<<std::endl;
		MathLib::mprint(log_stringstream_,dim_x_conc_, x_conc_,  "x");
		MathLib::mprint(log_stringstream_,dim_xdes_conc_, xdes_conc_, "xdes");
		MathLib::mprint(log_stringstream_,dim_u_conc_,u_conc_,  "u");
		MathLib::mprint(log_stringstream_,dim_udes_conc_,udes_conc_,"udes");
	}
	if(flag_show_controllertrace_){
		MathLib::mprint(log_stringstream_,dim_x_conc_, x1_, "x1");
	}

	//Excert Control Calculations
	unew(t,x_conc_,x1_,optvar_conc_);

	if(flag_show_controllerinfo_){
		log_stringstream_<<"Horizon thor:"<<thor_<<std::endl;
		log_stringstream_<<"Cmscgmres loop calc time: "<<ros::Time::now().toSec() - clock<<" sec\n"<<std::endl;
	}
	//Publish Computation time
	std_msgs::Float32 comptime;
	comptime.data=(ros::Time::now().toSec() - clock);
	comp_time_publisher_.publish(comptime);

	bool flag_control_isnan=false;
	//Check if output is NAN
	for(i=0;i<dim_u_conc_;i++){
		if(std::isnan(u_conc_[i])){
			flag_control_isnan=true;
			u_conc_[i]=1.0*rand()/(1.0*RAND_MAX);
			std::cout<<" !!! output "<<i<<" is nan => set randomly to "<<(double)u_conc_[i]<<std::endl;
		}
	}
	if(flag_control_isnan){
		//If nan was detected, reinitialize with random values.
		getOptimalInitialValues(x_conc_,optvar_conc_);
		log_stringstream_<<"Get initialized input"<<std::endl;
		MathLib::mprint(log_stringstream_,dim_u_conc_,u_conc_,  "u");
	}

	//	//Check if lagrange multipliers are positive
	//	for(i=u_conc_dim;i<dim_optvar_conc_;i++){
	//		if(optvar_conc[i]<0){
	//			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	//			printf("!! lagrange multiplier u[%d] should not be negative !!\n",i);
	//			//printf("!! Reinitialization in Progress 					!!\n");
	//			printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	//			//getOptimalInitialValues(x_conc,u);
	//			optvar_conc[i]=0;
	//		}
	//	}

	if(flag_save_controllerlog_){
		std::cout<<"logged to "<<log_filename_<<std::endl;
		log_file_.open(log_filename_, std::ios_base::app);
		log_file_<<log_stringstream_.str();
		log_file_.close();
	}
	std::cout<<log_stringstream_.str();
	log_stringstream_.str("");

	controlcounter_++;
}

void Cmscgmres::dhu0func(unsigned dimu, double *du0, double *dhu) {
	double du[dim_optvar_conc_], u[dim_optvar_conc_], hu[dim_optvar_conc_];
	mmulsc(1,dim_optvar_conc_, du0, hdir_, du);
	madd(1,dim_optvar_conc_, u0_, du, u);
	hufunc(tsim0_, x0_, lmd0_, u, hu);
	msub(1,dim_optvar_conc_, hu, hu0_, dhu);
	mdivsc(1,dim_optvar_conc_, dhu, hdir_, dhu);
}

void Cmscgmres::adufunc(unsigned n, double *du, double *adu) {

	mmulsc(1, n, du, hdir_, dutmp_);	//dutmp	=h*du
	madd(1, n, U_[0], dutmp_, Uk_[0]);	//Uk[0]	=U[0]+h*du

	//		/**************************************************
	//		 * Check that lagrange multipliers
	//		 **************************************************/
	//		for(int j=0;j<nhor_;j++){
	//			for(int k=0;k<dim_optvar_conc_;k++){
	//				if((k==8)||(k==9)||(k==10)||(k==11)){
	//					if(U_[j][k]<0){
	//						U_[j][k]=0;
	//					}
	//				}
	//			}
	//		}

	Hfunc(Uk_, G1s_, x1s_, ts_, Hk_);		//Hk		=H(G1s,x1s,ts)
	Ffunc(Uk_, Hk_, x1s_, ts_, Fk_);		//Fk		=F(Hk,x1s,ts)
	msub(1, n, Fk_[0], F1s_[0], adu);	//dudt	=Fk[0]-F1s[0]=Ffunc(Hfunc(U[0]+h*du, G1s, x1s, ts),x1s,ts)-Ffunc(U, X, x1s, ts);

	if(flag_show_controllertrace_){
		MathLib::mprint(log_stringstream_,2*nhor_,dim_x_conc_, X_, "X");
		MathLib::mprint(log_stringstream_,nhor_,dim_optvar_conc_, Fk_, "Fk");
		MathLib::mprint(log_stringstream_,nhor_,dim_x_conc_, Hk_, "Hk");
		MathLib::mprint(log_stringstream_,dim_f_, duvec_, "duvec");
		MathLib::mprint(log_stringstream_,dim_g_, dxvec_, "dxvec");
	}

}

void Cmscgmres::unew(double t, double x[], double x1[], double u[])
{
	int i;
	double x2[dim_x_conc_];

	ts_ = t + hdir_;
#ifdef HDIR_EQ_HT
	mmov(1,dim_x_conc_, x1, x1s_);
#else
	mmulsc(1,dim_x_conc_, x, onemhdir_, x2);		//x2=x*(1-hdir/ht)
	mmulsc(1,dim_x_conc_, x1, hdirbht_ , x1s_);	//x1s=x1*hdir/ht
	madd(1,dim_x_conc_, x2, x1s_, x1s_);			//x1s=x*(1-hdir/ht)+x1*hdir/ht=x+(x1-x)*hdir/ht
#endif

	//F[0]			=Hu(xk,		Xk[1],	Uk[0])	=Hu(xk,Lk[0],Uk[0])
	//F[1::nhor_-1]	=Hu(Xk[2i],	Xk[2i+3],Uk[i+1]) =HU(Xk[i],Lk[i+1],Uk[i]) //i=1...nhor_-2
	Ffunc(U_, X_, x, t, F_);   //ok
	Gfunc(U_, X_, x, t, G_);	//ok
	Ffunc(U_, X_, x1s_, ts_, F1s_);
	Gfunc(U_, X_, x1s_, ts_, G1s_);
	mmulsc(1, dim_f_, F_[0], onemzetahdir_, F1_[0]);//F1=F*(1-z*hdir/ht)
	mmulsc(1, dim_g_, G_[0], onemzetahdir_, G1_[0]);//G1=G*(1-z*hdir/ht)

	Hfunc(U_, G1_, x1s_, ts_, H1_);
	Ffunc(U_, H1_, x1s_, ts_, F2_);
	msub(1, dim_f_, F1_[0], F2_[0], bvec_);

	/*printf("%g\n", mvinner(dimf, F[0], F[0]) + mvinner(dimg, G[0], G[0]));*/
	sum_ += mvinner(dim_f_, F_[0], F_[0]) + mvinner(dim_g_, G_[0], G_[0]);
	if(flag_show_controllertrace_){
		printf("  SUM  = %f\n",sum_);
		printf("  <F,F>= %f\n",mvinner(dim_f_, F_[0], F_[0]));
		printf("  <G,G>= %f\n",mvinner(dim_g_, G_[0], G_[0]));
	}

	//	if(flag_show_controllertrace_){
	//	printf("isim=%d, t=%g \n", controlcounter_, (float)t);
	//	printf("taut=%10g, x0=%10g,                u0=%10g \n",
	//			(float)tau_[0], (float)x[0], (float)U_[0][0]);
	//	for(i=0; i<nhor_-1; i++){
	//		printf("taut=%10g, x0=%10g, l0=%10g, u0=%10g \n",
	//				(float)tau_[i+1], (float)X_[2*i][0], (float)X_[2*i+1][0], (float)U_[i+1][0], (float)F[i][0]);
	//	}
	//	printf("taut=%10g, x0=%10g, l0=%10g \n",
	//			(float)tau_[nhor_], (float)X_[2*nhor_-2][0], (float)X_[2*nhor_-1][0]);
	//}

	nfgmres(&Cmscgmres::adufunc, dim_f_, bvec_, duvec_, kmax_, errvec_);

	if(flag_show_controllertrace_){
		MathLib::mprint(log_stringstream_,kmax_, errvec_, "errvec");
	}

	mmulsc(1, dim_f_, duvec_, hdir_, U1_[0]);
	madd(1, dim_f_, U_[0], U1_[0], U1_[0]);
	Hfunc(U1_, G1_, x1s_, ts_, H1_);
	msub(1, dim_g_, H1_[0], X_[0], dxvec_); //dxvec=Hfunc(U+h*dU,G1,x1s,ts)-X

	for(i=0; i<dim_f_; i++){
		U_[0][i] += ht_ * duvec_[i];
#ifdef RESET_DU
		duvec_[i] = 0;   /**** Reset dU ****/
#endif
	}

	//	/**************************************************
	//	 * Check that lagrange multipliers
	//	 **************************************************/
	//	for(int j=0;j<nhor_;j++){
	//		for(int k=0;k<dim_optvar_conc_;k++){
	//			if((k==8)||(k==9)||(k==10)||(k==11)){
	//				if(U_[j][k]<0){
	//					U_[j][k]=0;
	//				}
	//			}
	//		}
	//	}

	for(i=0; i<dim_g_; i++){
		X_[0][i] += htbhdir_ * dxvec_[i];
#ifdef RESET_DX
		dxvec_[i] = 0;   /**** Reset dX ****/
#endif
	}
	mmov(1, dim_optvar_conc_, U_[0], u);

	if(flag_show_controllertrace_){
		MathLib::mprint(log_stringstream_,nhor_,dim_optvar_conc_, F_, "F");
		MathLib::mprint(log_stringstream_,nhor_,dim_optvar_conc_, F1_, "F1");
		MathLib::mprint(log_stringstream_,nhor_,dim_optvar_conc_, F1s_, "F1s");
		MathLib::mprint(log_stringstream_,nhor_,dim_optvar_conc_, F2_, "F2");
		MathLib::mprint(log_stringstream_,2*nhor_,dim_x_conc_, G_, "G");
		MathLib::mprint(log_stringstream_,2*nhor_,dim_x_conc_, G1_, "G1");
		MathLib::mprint(log_stringstream_,2*nhor_,dim_x_conc_, G1s_, "G1s");
		MathLib::mprint(log_stringstream_,nhor_,dim_x_conc_, H1_, "H1");
		MathLib::mprint(log_stringstream_,nhor_,dim_optvar_conc_, U_, "U");
		MathLib::mprint(log_stringstream_,2*nhor_,dim_x_conc_, X_, "X");
		MathLib::mprint(log_stringstream_,dim_optvar_conc_, u, "uout");
		MathLib::mprint(log_stringstream_,dim_f_, duvec_, "duvec");
		MathLib::mprint(log_stringstream_,dim_g_, dxvec_, "dxvec");
	}

}

/*--------------------------------------------------------
 * Linear Equation Subroutine
 * GMRES (Generalized Minimum Residual)
 * Cf. C.T.Kelley: Iterative Methods for Linear and Nonlinear Equations
 * T.Ohtsuka  '00/01/24
 * axfunc(unsigned n, double *x, double *ax): a function that gives A*x
 * n: dim x
 * b: the right-hand-side of the equation, A*x = b
 * x: initial guess, to be replaced with the solution
 * kmax: number of iteration
 * err: residual norms, |b - A*x_i| (i=1,...,n+1)
 * ----------------------------------------------------------*/
void Cmscgmres::nfgmres(funcptr_cgmresfunc axfunc, unsigned n, double *b,
		double *x, unsigned kmax, double *err) {
	int i,j,k;
	double rho, nu, normv_unorth,hr;
	double *cvec, *svec, *gvec, *tmpvec, **hmat, **vmat;

	cvec = defvector(kmax+1);
	svec = defvector(kmax+1);
	gvec = defvector(kmax+1);
	tmpvec = defvector(n);
	hmat = defmatrix(kmax+1,kmax+1);
	vmat = defmatrix(kmax+1, n);

	(this->*axfunc)(n, x, tmpvec);
	msub(1,n, b, tmpvec, tmpvec);
	rho = sqrt(mvinner(n, tmpvec, tmpvec));
	gvec[0] = rho;
	for(i=1; i<kmax+1; i++){
		gvec[i] = 0;
	}
	err[0] = rho;

	if(flag_show_controllerinfo_){
		printf("  Rho  =%f\n",rho);
	}

	//If rho=0. Given Control is optimal -> Do nothing
	if(rho!=0){

		if(std::isnan(rho)){
			rho=1/rtol_;
			if(flag_show_controllerinfo_){
				printf("GMRES: rho=nan => inequality violated=> set rho=1/rtol=%f \n",rho);
			}
		}

		mdivsc(1,n, tmpvec, rho, vmat[0]);

		for(k=0; ((k<kmax)&&(rho>rtol_)); k++){
			(this->*axfunc)(n, vmat[k], vmat[k+1]);
			normv_unorth=sqrt(mvinner(n, vmat[k+1], vmat[k+1]));
			/* Modified Gram-Schmidt */
			for(j=0; j<=k; j++){
				hmat[k][j] = mvinner(n, vmat[j], vmat[k+1]);
				mmulsc(1,n, vmat[j], hmat[k][j], tmpvec);
				msub(1,n, vmat[k+1], tmpvec, vmat[k+1]);
			}
			hmat[k][k+1] = sqrt(mvinner(n, vmat[k+1], vmat[k+1]));

			//REORTHOGONALIZE:: If norm_unorth + 0.001*norm_orth==1
			if((normv_unorth + .0001*hmat[k][k+1])==normv_unorth){
				if(flag_show_controllerinfo_){
					//					printf("GMRES::Reorthogonalize/n");
					printf("GMRES::Reorthogonalize as normv_unorth + .0001*hmat[k][k+1]==%f==%f==normv_unorth /n",(normv_unorth + .0001*hmat[k][k+1]),normv_unorth);
				}
				for(j=0; j<=k; j++){
					hr=mvinner(n, vmat[j], vmat[k+1]);
					hmat[k][j]=hmat[k][j]+hr;
					mmulsc(1,n, vmat[j], hr, tmpvec);
					msub(1,n, vmat[k+1], tmpvec, vmat[k+1]);
				}
				hmat[k][k+1] = sqrt(mvinner(n, vmat[k+1], vmat[k+1]));
			}

			/* No Breakdown? */
			if( hmat[k][k+1] != 0){
				mdivsc(1,n, vmat[k+1], hmat[k][k+1], vmat[k+1]);
			}
			else{
				/*Happy Breakdown*/
				mmov(1,n,vmat[k+1],x);
				if(flag_show_controllerinfo_){
					printf("gmress() :happy breakdown \n");
				}
				freevector(cvec);
				freevector(svec);
				freevector(gvec);
				freevector(tmpvec);
				freematrix(hmat);
				freematrix(vmat);

				return;
			}

			/* Givens Rotation */
			givrot(k, cvec, svec, hmat[k]);
			nu = sqrt(hmat[k][k] * hmat[k][k] + hmat[k][k+1] * hmat[k][k+1]);
			if( nu != 0 ){
				cvec[k] = hmat[k][k] / nu;
				svec[k] = - hmat[k][k+1] / nu;
				hmat[k][k] = cvec[k] * hmat[k][k] - svec[k] * hmat[k][k+1];
				hmat[k][k+1] = 0;
				givrot(1, cvec+k, svec+k, gvec+k);
			}
			else{
				if(flag_show_controllerinfo_){
					printf("nu is zero!\n");
				}
			}

			/* Residual Update */
			rho = fabs(gvec[k+1]);
			err[k+1] = rho;
		}


		/* Solve hmat * y = gvec (hmat: upper triangular) */
		for(i=k-1; i>=0; i--){
			for(nu=gvec[i], j=i+1; j<k; j++){
				nu -= hmat[j][i] * cvec[j];
			}
			cvec[i] = nu / hmat[i][i] ;
			/*		for(nu=0, j=i+1; j<k; j++){
			nu += hmat[j][i] * cvec[j];
		}
		cvec[i] = (gvec[i] - nu) / hmat[i][i] ;*/
		}
		/* Ans. */
		for(i=0; i<n; i++){
			for(nu=0, j=0; j<k; j++){
				nu += vmat[j][i] * cvec[j];
			}
			x[i] += nu;
		}
	}
	else{
		if(flag_show_controllerinfo_){
			printf("   CGMRES rho=0 ->optimal\n");
		}
	}
	freevector(cvec);
	freevector(svec);
	freevector(gvec);
	freevector(tmpvec);
	freematrix(hmat);
	freematrix(vmat);
}

void Cmscgmres::getOptimalInitialValues(double* x, double* u)
{
	//Initialize Vector values
	initVectors();

	int i, j;
	double r, b[dim_optvar_conc_], du0[dim_optvar_conc_], erru0[dim_optvar_conc_+1];
	lmd0_ = defvector(dim_x_conc_);
	hu0_ = defvector(dim_optvar_conc_);
	mmov(1,dim_x_conc_, x0_, x);
	phix(tsim0_, x0_, lmd0_);
	hufunc(tsim0_, x0_, lmd0_, u0_, hu0_);
#ifdef TRACE_ON
	printf("lmd0 = {%g", lmd0_[0]);
	for(j=1; j<dim_x_conc_; j++){
		printf(", %g", lmd0_[j]);
	}
	printf("} \n");
	printf("hu0 = {%g", hu0_[0]);
	for(j=1; j<dim_optvar_conc_; j++){
		printf(", %g", hu0_[j]);
	}
	printf("} \n");
#endif
	for(i=0; i<dim_optvar_conc_; i++) du0[i] = 0;
	r = sqrt(mvinner(dim_optvar_conc_, hu0_, hu0_));
	i=0;
	while( (r > rtol_) && (i < 100) ){
		for(j=0; j<dim_optvar_conc_; j++) {
			b[j] = -hu0_[j];
		}
		nfgmres(&Cmscgmres::dhu0func, dim_optvar_conc_, b, du0, dim_optvar_conc_, erru0);
#ifdef TRACE_ON
		printf("i=%d ,  r=%g \n", i, r);
		printf("u0 = {%g", u0[0]);
		for(j=1; j<dim_optvar_conc_; j++){
			printf(", %g", u0[j]);
		}
		printf("} \n");
		printf("erru0 = ");
		for(j=0; j<=dim_optvar_conc_; j++){
			printf("%g ", (float)erru0[j]);
		}
		printf("\n");
#endif
		madd(1,dim_optvar_conc_, u0_, du0, u0_);
		hufunc(tsim0_, x0_, lmd0_, u0_, hu0_);
		//		MathLib::mprint(log_stringstream_,dim_output, u0, "u0");
		//		MathLib::mprint(log_stringstream_,dim_output, hu0, "hu0");
		r = sqrt(mvinner(dim_optvar_conc_, hu0_, hu0_));
		i++;
	}
#ifdef TRACE_ON
	printf("i=%d ,  r=%g \n", i, r);
	printf("u0 = {%g", u0[0]);
	for(j=1; j<dim_optvar_conc_; j++){
		printf(", %g", u0[j]);
	}
	printf("} \n\n");
#endif
	mmov(1,dim_optvar_conc_, u0_, u);
	for(i=0; i<nhor_; i++){
		mmov(1,dim_optvar_conc_, u0_, U_[i]);
		mmov(1,dim_x_conc_, x0_, X_[2*i]);
		mmov(1,dim_x_conc_, lmd0_, X_[2*i+1]);
	}
	for(i=0; i<dim_f_; i++){
		duvec_[i] = 0;
	}
	for(i=0; i<dim_g_; i++){
		dxvec_[i] = 0;
	}

	freevector(lmd0_);
	freevector(hu0_);
}


/*-------------- F(U,X,x,t)-------------- */
void Cmscgmres::Ffunc(double **Uk, double **Xk, double *xk, double tk, double **Fk)
{
	int i;
	double tauf, taut;

	tauf = thor_ * (1. - exp(-alpha_ * tk) );
	htau_ = tauf / nhor_;
	taut = tk;
	tau_[0] = taut;
	//F[0]			=Hu(xk,		Xk[1],	Uk[0])	=Hu(xk,Lk[0],Uk[0])
	hufunc(taut, xk, Xk[1], Uk[0], Fk[0]);
	//F[1::nhor_-1]	=Hu(Xk[2i],	Xk[2i+3],Uk[i+1]) =HU(Xk[i],Lk[i+1],Uk[i]) //i=1...nhor_-2
	for(taut = tk+ htau_, i=0;i<nhor_-1;i++){
		tau_[i+1]=taut;
		hufunc(taut, Xk[2*i], Xk[2*i+3], Uk[i+1], Fk[i+1]);
		taut += htau_;
	}
	tau_[nhor_] = taut;
	//MathLib::mprint(log_stringstream_,nhor_,dim_output, Fk, "Ffunc:Fk");
}

/*-------------- G(U,X,x,t)-------------- */
void Cmscgmres::Gfunc(double **Uk, double **Xk, double *xk, double tk, double **Gk)
{
	int i;
	double tauf, taut, linp[dim_x_conc_+dim_optvar_conc_];
	double xerr[dim_x_conc_], lerr[dim_x_conc_], gxr[dim_x_conc_], glr[dim_x_conc_], phixn[dim_x_conc_];
	//initalization of phixn
	for(int i=0;i<dim_x_conc_;i++){
		phixn[i]=0;
	}

	//	double tauf, taut, linp[16];
	//	double xerr[12], lerr[12], gxr[12], glr[12], phixn[12];


	tauf = thor_ * (1. - exp(-alpha_ * tk) );
	htau_ = tauf / nhor_;
	taut = tk;

	//Gx[0]=(Xk[0]-xk)-h*f(xk,Uk[0])
	msub(1,dim_x_conc_, Xk[0], xk, xerr);		//xerr=Xk[0]-xk
	xpfunc(taut, xk, Uk[0], gxr);		//gxr=f(xk,Uk[0])
	mmulsc(1,dim_x_conc_, gxr, htau_, gxr);	//gxr=h*f(xk,Uk[0])
	msub(1,dim_x_conc_, xerr, gxr, Gk[0]);	//gxr=(Xk[0]-xk)-h*f(xk,Uk[0])

	//Gx[2i+2]=(Xk[2i+2]-Xk[2i])-h*f(Xk[2i],Uk[i+1])
	//Gl[2i+1]=(Xk[2i+1]-Xk[2i+3])+h*(-dHdx)(Xk[2i+3],(Xk[2i],Uk[i+1]))
	for(taut = tk+htau_, i=0;i<nhor_-1;i++){
		msub(1,dim_x_conc_, Xk[2*i+2], Xk[2*i], xerr);	//xerr=Xk[2i+2]-Xk[2i]
		msub(1,dim_x_conc_, Xk[2*i+1], Xk[2*i+3], lerr);//lerr=Xk[2i+1]-Xk[2i+3]//Lambdaerror
		xpfunc(taut, Xk[2*i], Uk[i+1], gxr);		//gxr=f(Xk[2i],Uk[i+1])
		mmov(1,dim_x_conc_, Xk[2*i],linp);
		mmov(1,dim_optvar_conc_, Uk[i+1], linp+dim_x_conc_);
		lpfunc(taut, Xk[2*i+3], linp, glr);		//glr=(-dHdx)(Xk[2i+3],(Xk[2i],Uk[i+1]))=(-dHdx)(lmd[k],x[k],Uk[k+1]))
		mmulsc(1,dim_x_conc_, gxr, htau_, gxr);				//gxr=h*f(Xk[2i],Uk[i+1])
		mmulsc(1,dim_x_conc_, glr, htau_, glr);				//glr=h*(-dHdx)(Xk[2i+3],(Xk[2i],Uk[i+1]))
		msub(1,dim_x_conc_, xerr, gxr, Gk[2*i+2]);		//Gk[2i+2]=(Xk[2i+2]-Xk[2i])-h*f(Xk[2i],Uk[i+1])
		madd(1,dim_x_conc_, lerr, glr, Gk[2*i+1]);		//Gk[2i+1]=(Xk[2i+1]-Xk[2i+3])+h*(-dHdx)(Xk[2i+3],(Xk[2i],Uk[i+1]))
		taut += htau_;
	}
	phix(taut, Xk[2*nhor_-2], phixn);						//phixn=dVdx(Xk[2*nhor_-2])
	msub(1,dim_x_conc_, Xk[2*nhor_-1], phixn, Gk[2*nhor_-1]);	//Gk[2*nhor_-1]=Xk[2*nhor_-1]-dVdx(Xk[2*nhor_-2])
}

/*-------------- H(U,Z,x,t)-------------- */
void Cmscgmres::Hfunc(double **Uk, double **Zk, double *xk, double tk, double **Hk)
{
	int i;
	double tauf, taut, linp[dim_x_conc_+dim_optvar_conc_];
	//	double tauf, taut, linp[16];


	tauf = thor_ * (1. - exp(-alpha_ * tk) );
	htau_ = tauf / nhor_;
	taut = tk;

	this->nfeulerinp(&Cmscgmres::xpfunc, taut, xk, Uk[0], htau_, dim_x_conc_, Hk[0]);	//Hk[0]=xk+h*f(xk,Uk[0])
	madd(1,dim_x_conc_, Hk[0], Zk[0], Hk[0]);								//Hk[0]=Hk[0]+Zk[0]=xk+h*f(xk,Uk[0])+Z[k]

	for(taut = tk + htau_, i=0; i < nhor_-1; taut += htau_, i++){
		this->nfeulerinp(&Cmscgmres::xpfunc, taut, Hk[2*i], Uk[i+1], htau_, dim_x_conc_, Hk[2*i+2]);	//Hk[2*i+2]=Hk[2*i]+h*f(Hk[2*i],Uk[i+1])
		madd(1,dim_x_conc_, Hk[2*i+2], Zk[2*i+2], Hk[2*i+2]);								//Hk[2*i+2]=Hk[2*i+2]+Zk[2*i+2]=Hk[2*i]+h*f(Hk[2*i],Uk[i+1])+Zk[2*i+2]
	}
	phix(taut, Hk[2*nhor_-2], Hk[2*nhor_-1]);	 					//Hk[2*nhor_-1]=dVdx(Hk[2*nhor_-2])
	madd(1, dim_x_conc_, Hk[2*nhor_-1], Zk[2*nhor_-1], Hk[2*nhor_-1]);	//Hk[2*nhor_-1]=Hk[2*nhor_-1]+Zk[2*nhor_-1]=dVdx(Hk[2*nhor_-2])+Zk[2*nhor_-1]

	for(i = nhor_-2; i >= 0; i--){
		mmov(1,dim_x_conc_, Hk[2*i], linp);
		mmov(1,dim_optvar_conc_, Uk[i+1], linp+dim_x_conc_);
		this->nfeulerinp(&Cmscgmres::lpfunc,taut,Hk[2*i+3],linp,-htau_,dim_x_conc_,Hk[2*i+1]);//Hk[2*i+1]=Hk[2*i+3]-h*(-dHdx)(Hk[2*i+3],Hk[2*i],Uk[i+1])//(-dHdx)(lmd,x,u)
		madd(1,dim_x_conc_, Hk[2*i+1], Zk[2*i+1], Hk[2*i+1]);		//Hk[2*i+1]=Hk[2*i+3]-h*(-dHdx)(Hk[2*i+3],Hk[2*i],Uk[i+1])+Zk[2*i+1]
		taut -= htau_;
	}
}

/*---- a[m][n] -> b[m][n] ----*/
void Cmscgmres::mmov( int n, double *a, double *b )
{
	for(int	j = 0; j < n; j++){
		b[j] = a[j] ;
	}
}
/*---- a[m][n] -> b[m][n] ----*/
void Cmscgmres::mmov( int m, int n, double *a, double *b )
{
	int	i, j, tmp ;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++){
			tmp = i*n + j ;
			b[tmp] = a[tmp] ;
		}
}

/*---- a[m][n] + b[m][n] -> c[m][n] ----*/
void Cmscgmres::madd( int m, int n, double *a, double *b, double *c )
{
	int	i, j, tmp ;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++) {
			tmp = i*n + j ;
			c[tmp] = a[tmp] + b[tmp] ;
		}
}

/*---- a[m][n] - b[m][n] -> c[m][n] ----*/
void Cmscgmres::msub( int m, int n, double *a, double *b, double *c )
{
	int	i, j, tmp ;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++) {
			tmp = i*n + j ;
			c[tmp] = a[tmp] - b[tmp] ;
		}
}

/*---- k * a[m][n] -> b[m][n] ----*/
void Cmscgmres::mmulsc( int m, int n, double *a, double k, double *b )
{
	int	i, j, tmp;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++){
			tmp = i*n + j ;
			b[tmp] = k * a[tmp];
		}
}

/*---- a[m][n] -> -a[m][n] ----*/
void Cmscgmres::mminus( int m, int n, double *a, double *b )
{
	int	i, j, tmp;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++){
			tmp = i*n + j ;
			b[tmp] = -a[tmp];
		}
}

/*---- a[m][n] / k -> b[m][n] ----*/
void Cmscgmres::mdivsc( int m, int n, double *a, double k, double *b )
{
	int	i, j, tmp;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++){
			tmp = i*n + j ;
			b[tmp] = a[tmp] / k;
		}
}

/*---- Inner Product of a[m] and b[m] ----*/
double	Cmscgmres::mvinner( int m, double *a, double *b )
{
	int	i;
	double tmp;
	tmp =0.;
	for(i=0; i<m; i++)
		tmp += a[i] * b[i];
	return tmp;
}

/*---- Define an n-Dimensional Vector ----*/
double *Cmscgmres::defvector(int n)
{
	double *v;
	v = (double *)malloc( (size_t)( n * sizeof(double) ) );
	//v= new double [n];
	if(!v) {
		printf("defvector() : allocation failure. \n");
		exit(1);
	}
	for(unsigned i=0;i<n;i++)
		v[i]=0;
	return v;
}

void Cmscgmres::freevector(double *v)
{
	std::free(v);
}

/*---- Define an n by m Matrix ----*/
double **Cmscgmres::defmatrix(int n, int m)
{
	unsigned i;
	double **a;

	a = (double **)malloc( ( n * sizeof(double*) ) );
	//a = new double* [n];
	if(!a) {
		printf("defmatrix() : allocation failure. \n");
		exit(1);
	}
	a[0] = (double *)malloc( ( n * m * sizeof(double) ) );
	//a[0] = new double [n*m];
	if(!a[0]) {
		printf("defmatrix() : allocation failure. \n");
		exit(1);
	}
	for(i=0; i<n-1; i++) a[i+1] = a[i] + m;
	return a;
}

void Cmscgmres::freematrix(double **a)
{
	//printf("free matrix\n");
	std::free(a[0]);
	std::free(a);
}

///*---- Define an n-Dimensional Vector ----*/
//double *Cmscgmres::defvector(int n)
//{
//	auto vec=new double[n];
//	return vec;
//}
//
//void Cmscgmres::freevector(double *v)
//{
//	delete v;
//}
//
///*---- Define an n by m Matrix ----*/
//double **Cmscgmres::defmatrix(int n, int m)
//{
//	double** mat = new double*[n];
//	for(int i=0;i<n;i++)
//		mat[i]=new double[m];
//	return mat;
//}
//
//void Cmscgmres::freematrix(double **a)
//{
//	delete[] a;
//}



/*---- k Givens Roations used in GMRES ----*/
void Cmscgmres::givrot(unsigned k, double *c, double *s, double *v) {
	unsigned i;
	double w1, w2;
	for (i = 0; i < k; i++) {
		w1 = c[i] * v[i] - s[i] * v[i + 1];
		w2 = s[i] * v[i] + c[i] * v[i + 1];
		v[i] = w1;
		v[i + 1] = w2;
	}
}

/*--------------------------------------------------------
 * Simultaneous Ordinaly Diferential Equation Subroutine
 * Runge-Kutta_Gill Method	(Original by T.Murayama)
 *
 * ---- Variation of rkg() : dx/dt is also returned. ----
 * ----  (for Start of the Adams method)  ----
 * ----------------------------------------------------------*/
void Cmscgmres::nfrkginpex(funcPtr_func func, double x, double* y, double* u,	double h, unsigned dim, double* ans, double* fxy)
{
	int i;
	double fval[dim],k1[dim],k2[dim],k3[dim],k4[dim],
	yp1[dim],yp2[dim],yp3[dim],
	q1[dim],q2[dim],q3[dim],
	c1 = 0.2928932188134528,
	c2 = 0.1213203435596426,
	c3 = 0.5857864376269054,
	c4 = 1.707106781186548,
	c5 = 4.121320343559643,
	c6 = 3.414213562373097;

	/*	fval = (double *)malloc( (size_t)( dim * sizeof(double) ) );
	k1   = (double *)malloc( (size_t)( dim * sizeof(double) ) );
	k2   = (double *)malloc( (size_t)( dim * sizeof(double) ) );
	k3   = (double *)malloc( (size_t)( dim * sizeof(double) ) );
	k4   = (double *)malloc( (size_t)( dim * sizeof(double) ) );
	 */

	(this->*func)(x,y,u,fxy);
	for(i = 0; i < dim; i++)
	{
		k1[i] = h * fxy[i];
		yp1[i] = y[i] + 0.5 * k1[i];
		q1[i] = k1[i];
	}
	(this->*func)(x + 0.5 * h,yp1,u,fval);
	for(i = 0; i < dim; i++)
	{
		k2[i] = h * fval[i];
		yp2[i] = yp1[i] + c1 * (k2[i] - q1[i]);;
		q2[i] = c2 * q1[i] + c3 * k2[i];
	}
	(this->*func)(x + 0.5 * h,yp2,u,fval);
	for(i = 0; i < dim; i++)
	{
		k3[i] = h * fval[i];
		yp3[i] = yp2[i] + c4 * (k3[i] - q2[i]);
		q3[i] = -c5 * q2[i] + c6 * k3[i];
	}
	(this->*func)(x + h,yp3,u,fval);
	for(i = 0; i < dim; i++)
	{
		k4[i] = h * fval[i];
		ans[i] = yp3[i] + k4[i] / 6.0 - q3[i] / 3.0;
	}
}

/*--------------------------------------------------------
 * Simultaneous Ordinaly Diferential Equation Subroutine
 * Adams Method (Predictor-Corrector Method)
 * Predictor: Adams-Bashforth
 * Corrector: Adams-Moulton
 * T.Ohtsuka  '92/10/24
 * ----------------------------------------------------------*/
void Cmscgmres::nfadamsinp(funcPtr_func func, double x, double* y, double* u,	double* f1, double* f2, double* f3, double h, unsigned dim,double* ans)
{
	int i;
	double y1[dim], fval[dim], cd;
	/*	double *y1, *fval, cd;

	fval = (double *)malloc( (size_t)( dim * sizeof(double) ) );
	y1   = (double *)malloc( (size_t)( dim * sizeof(double) ) ); */
	cd = h / 24.;

	/*---- Adams-Bashforth Predictor ----*/
	(this->*func)(x,y,u,fval);
	for(i=0; i<dim; i++)
	{
		y1[i] = 55.* fval[i] -59.* f1[i] +37.* f2[i] -9.* f3[i];
		y1[i] *= cd;
		y1[i] += y[i];
		f3[i] = f2[i]; f2[i] = f1[i]; f1[i] = fval[i]; /* Shift */
	}

	/*---- Adams-Moulton Corrector ----*/
	(this->*func)(x+h,y1,u,fval);
	for(i=0; i<dim; i++)
	{
		y1[i] = 9.* fval[i] +19.* f1[i] -5.* f2[i] + f3[i];
		y1[i] *= cd;
		ans[i] = y1[i] + y[i];
	}

	/*	free(fval);
	free(y1); */
}

/*--------------------------------------------------------
 * Simultaneous Ordinaly Diferential Equation Subroutine
 * Euler Method (Forward Differentce)
 * T.Ohtsuka  '99/12/19
 * ----------------------------------------------------------*/
void Cmscgmres::nfeulerinp(funcPtr_func func, double x, double* y, double* u,	double h, unsigned dim, double* ans)
{
	int i;
	double fval[dim];

	(this->*func)(x,y,u,fval);
	for(i=0; i<dim; i++)
		ans[i] = y[i] + h * fval[i];
}

