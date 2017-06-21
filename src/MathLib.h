/**
 * @file    MathLib.h
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

#ifndef MATHLIB_H_
#define MATHLIB_H_

#include <ros/ros.h>
#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace MathLib{

/*---- print array ----*/
void mprint(unsigned nstate, double *a, const char* name);

void mprint(std::vector<double>a, const char* name);

/*---- print matrix ----*/
void mprint(unsigned nhor, unsigned nstate, double **a, const char* name);

/*---- print array ----*/
void mprint(std::ostream& ss, unsigned nstate, double *a, const char* name);
/*---- print matrix ----*/
void mprint(std::ostream& ss, unsigned nhor, unsigned nstate, double **a, const char* name);



/*---- a[n] -> b[n] ----*/
inline void minit(double *b, unsigned n){
	for(int j = 0; j < n; j++){
		b[j] =0;
	}
}


/*---- a[n] -> b[n] ----*/
inline void mmov(double *b, unsigned n, std::vector<double> a)
{    unsigned j;
for(j = 0; j < n; j++){
	b[j] = a[j] ;
}
}

/*---- a[n] -> b[n] ----*/
inline void mmov(double *b, unsigned n, double *a )
{    unsigned j;
for(j = 0; j < n; j++){
	b[j] = a[j] ;
}
}

/*---- a[m][n] -> b[m][n] ----*/
inline void mmov( double **b, unsigned m, unsigned n, double **a )
{
	unsigned	i, j, tmp ;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++){
			b[i][j] = a[i][j];
		}
}

/*---- a[m][n] -> b[m][n] ----*/
inline void mmov(  double *b, unsigned m, unsigned n, double *a )
{
	unsigned	i, j, tmp ;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++){
			tmp = i*n + j ;
			b[tmp] = a[tmp] ;
		}
}
/*---- a[m][n] + b[m][n] -> c[m][n] ----*/
inline void madd(double *c, unsigned m, unsigned n, double *a, double *b )
{
	unsigned	i, j, tmp ;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++) {
			tmp = i*n + j ;
			c[tmp] = a[tmp] + b[tmp] ;
		}
}
/*---- a[m][n] + b[m][n] -> c[m][n] ----*/
inline void madd(unsigned n, double *a, double *b,double *c)
{
	unsigned	j;
	for(j = 0; j < n; j++) {
		c[j] = a[j] + b[j] ;
	}
}
/*---- a[m] - b[m] -> c[m] ----*/
inline void msub( double *c, unsigned m,double *a, double *b )
{
	unsigned	i, j;
	for(i = 0; i < m; i++){
		c[i] = a[i] - b[i] ;
	}
}
/*---- a[m][n] - b[m][n] -> c[m][n] ----*/
inline void msub( double *c, unsigned m, unsigned n, double *a, double *b )
{
	unsigned	i, j, tmp ;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++) {
			tmp = i*n + j ;
			c[tmp] = a[tmp] - b[tmp] ;
		}
}
/*---- k * a[m][n] -> b[m][n] ----*/
inline void mmulsc( double *b, unsigned m, unsigned n, double *a, double k )
{
	unsigned	i, j, tmp;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++){
			tmp = i*n + j ;
			b[tmp] = k * a[tmp];
		}
}
/*---- a[m][n] / k -> b[m][n] ----*/
inline void mdivsc( double *b, unsigned m, unsigned n, double *a, double k )
{
	unsigned	i, j, tmp;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++){
			tmp = i*n + j ;
			b[tmp] = a[tmp] / k;
		}
}








/*---- a[n] -> b[n] ----*/
inline void mmov(unsigned n, std::vector<double> a,double *b)
{    unsigned j;
for(j = 0; j < n; j++){
	b[j] = a[j] ;
}
}

/*---- a[n] -> b[n] ----*/
inline void mmov(unsigned n, double *a,double *b )
{    unsigned j;
for(j = 0; j < n; j++){
	b[j] = a[j] ;
}
}

/*---- a[m][n] -> b[m][n] ----*/
inline void mmov( unsigned m, unsigned n, double **a, double **b )
{
	unsigned	i, j, tmp ;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++){
			b[i][j] = a[i][j];
		}
}

/*---- a[m][n] -> b[m][n] ----*/
inline void mmov( unsigned m, unsigned n, double *a, double *b )
{
	unsigned	i, j, tmp ;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++){
			tmp = i*n + j ;
			b[tmp] = a[tmp] ;
		}
}
/*---- a[m][n] + b[m][n] -> c[m][n] ----*/
inline void madd(unsigned m, unsigned n, double *a, double *b,double *c )
{
	unsigned	i, j, tmp ;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++) {
			tmp = i*n + j ;
			c[tmp] = a[tmp] + b[tmp] ;
		}
}
/*---- a[m] - b[m] -> c[m] ----*/
inline void msub( unsigned m,double *a, double *b,double *c )
{
	unsigned	i, j;
	for(i = 0; i < m; i++){
		c[i] = a[i] - b[i] ;
	}
}
/*---- a[m][n] - b[m][n] -> c[m][n] ----*/
inline void msub( unsigned m, unsigned n, double *a, double *b,double *c )
{
	unsigned	i, j, tmp ;
	for(i = 0; i < m; i++)
		for(j = 0; j < n; j++) {
			tmp = i*n + j ;
			c[tmp] = a[tmp] - b[tmp] ;
		}
}
/*---- k * a[m][n] -> b[m][n] ----*/
inline void mmulsc( unsigned m, unsigned n, double *a, double k,double *b )
{
	unsigned	i, j, tmp;
	for(i = 0; i < m; i++){
		for(j = 0; j < n; j++){
			tmp = i*n + j ;
			b[tmp] = k * a[tmp];
		}
	}
}
/*---- k * a[m][n] -> b[m][n] ----*/
inline void mmulsc( unsigned n, double *a, double k,double *b )
{
	unsigned j;
	for(j = 0; j < n; j++){
		b[j] = k * a[j];
	}
}
/*---- a[m][n] / k -> b[m][n] ----*/
inline void mdivsc( unsigned m, unsigned n, double *a, double k,double *b )
{
	unsigned	i, j, tmp;
	for(i = 0; i < m; i++){
		for(j = 0; j < n; j++){
			tmp = i*n + j ;
			b[tmp] = a[tmp] / k;
		}
	}
}







/*---- Inner Product of a[m] and b[m] ----*/
inline double mvinner(  unsigned m, double *a, double *b)
{
	unsigned	i;
	double tmp;
	tmp =0.;
	for(i=0; i<m; i++)
		tmp += a[i] * b[i];
	return tmp;
}

/*---- Define an n-Dimensional Vector ----*/
inline bool *defboolvector(bool n)
{
	bool *v;
	v = (bool *)malloc( (size_t)( n * sizeof(bool) ) );
	//v= new double [n];
	if(!v) {
		printf("defvector() : allocation failure. \n");
		exit(1);
	}
	for(unsigned i=0;i<n;i++)
		v[i]=false;
	return v;
}

/*---- Define an n-Dimensional Vector ----*/
inline double *defvector(unsigned n)
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

/*---- Define an n-Dimensional Vector ----*/
inline double *defvector(unsigned n,const char* name)
{
	double *v;
	v = (double *)malloc( ( n * sizeof(double) ) );
	//v = new double [n];
	if(!v) {
		printf("defvector() of %s: allocation failure. \n",name);
		exit(1);
	}
	for(unsigned i=0;i<n;i++)
		v[i]=0;
	return v;
}

inline double *defvector(unsigned n,std::vector<double> initvector)
{
	double *v;
	v = (double *)malloc( ( n * sizeof(double) ) );
	//v = new double [n];
	if(!v) {
		printf("defvector(): allocation failure. \n");
		exit(1);
	}
	if(n==(initvector.size())){
		for(unsigned i=0;i<n;i++){
			v[i]=initvector[i];
		}
	}
	else{
		printf("defvector(): initialization failure. \n");
		exit(1);
	}
	return v;
}


/*---- Define an n-Dimensional Vector ----*/
inline double *defvector(unsigned n,double* initarray,const char* name)
{
	double *v;
	v = (double *)malloc( ( n * sizeof(double) ) );
	//v = new double [n];
	if(!v) {
		printf("defvector() of %s: allocation failure. \n",name);
		exit(1);
	}
	for(unsigned i=0;i<n;i++)
		v[i]=initarray[i];
	return v;
}

/*---- Define an n-Dimensional Vector ----*/
inline double *defvector(unsigned n,double* initarray)
{
	double *v;
	v = (double *)malloc( ( n * sizeof(double) ) );
	//v = new double [n];
	if(!v) {
		printf("defvector(): allocation failure. \n");
		exit(1);
	}
	for(unsigned i=0;i<n;i++)
		v[i]=initarray[i];
	return v;
}

/*---- Define an n-Dimensional Vector ----*/
inline int *def_int_vector(unsigned n,const char* name)
{
	int *v;
	v = (int *)malloc( ( n * sizeof(int) ) );
	//v = new double [n];
	if(!v) {
		printf("defvector() of %s: allocation failure. \n",name);
		exit(1);
	}
	return v;
}

/*---- Define an n-Dimensional Vector ----*/
inline int *def_int_vector(unsigned n,double* initarray,const char* name)
{
	int *v;
	v = (int *)malloc( ( n * sizeof(int) ) );
	//v = new double [n];
	if(!v) {
		printf("defvector() of %s: allocation failure. \n",name);
		exit(1);
	}
	for(unsigned i=0;i<n;i++)
		v[i]=initarray[i];
	return v;
}


/*---- Define an n by m Matrix ----*/
inline double **defmatrix(unsigned n, unsigned m)
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

inline double **defmatrix(unsigned n, unsigned m,const char* name)
{
	unsigned i;
	double **a;

	a = (double **)malloc( ( n * sizeof(double*) ) );
	//a = new double* [n];
	if(!a) {
		printf("defmatrix() of %s: allocation failure. \n",name);
		exit(1);
	}
	a[0] = (double *)malloc( ( n * m * sizeof(double) ) );
	//a[0] = new double [n*m];
	if(!a[0]) {
		printf("defmatrix() of %s: allocation failure. \n",name);
		exit(1);
	}
	for(i=0; i<n-1; i++) a[i+1] = a[i] + m;
	return a;
}
inline int **def_int_matrix(unsigned n, unsigned m,const char* name)
{
	unsigned i;
	int **a;

	a = (int **)malloc( ( n * sizeof(int*) ) );
	//a = new double* [n];
	if(!a) {
		printf("defmatrix() of %s: allocation failure. \n",name);
		exit(1);
	}
	a[0] = (int *)malloc( ( n * m * sizeof(int) ) );
	//a[0] = new double [n*m];
	if(!a[0]) {
		printf("defmatrix() of %s: allocation failure. \n",name);
		exit(1);
	}
	for(i=0; i<n-1; i++) a[i+1] = a[i] + m;
	return a;
}


inline void interpolatePolynomialOrder1(unsigned n,double tstart, double* startpoint, double tend, double* endpoint,double tint,double* intpoint)
{	unsigned i;
double timefactor=(tint-tstart)/(tend-tstart);
for(i=0;i<n;i++)
	intpoint[i]=(endpoint[i]-startpoint[i])/timefactor+startpoint[i];
}


inline void interpolatePolynomialOrder1(unsigned dimtime,unsigned dimstates,double* startpoint, double* endpoint,double** intmatrix)
{	unsigned i,j;
double timefactor;
for(j=0;j<dimtime;j++){ ///Every Column
	timefactor=(((float)j)/((float)(dimtime-1)));
	for(i=0;i<dimstates;i++){ ///Every Row
		intmatrix[j][i]=(endpoint[i]-startpoint[i])*timefactor+startpoint[i];
	}
}
}

inline void interpolatePolynomialOrder5(unsigned n,double tstart, double* startpoint, double tend, double* endpoint,double tint,double* intpoint)
{	unsigned i;
double timefactor=(tint-tstart)/(tend-tstart);
for(i=0;i<n;i++)
	intpoint[i]=startpoint[i]+(endpoint[i]-startpoint[i])*(10*pow(timefactor,3)-15*pow(timefactor,4)+6*pow(timefactor,5));
}

inline void interpolatePolynomialOrder5(unsigned dimtime,unsigned dimstates,double* startpoint, double* endpoint,double** intmatrix)
{	unsigned i,j;
double timefactor, polynomialfactor;
for(j=0;j<dimtime;j++){ ///Every Column
	timefactor=(((float)j)/((float)(dimtime-1)));
	polynomialfactor=(10*pow(timefactor,3)-15*pow(timefactor,4)+6*pow(timefactor,5));
	for(i=0;i<dimstates;i++){ ///Every Row
		intmatrix[j][i]=startpoint[i]+(endpoint[i]-startpoint[i])*polynomialfactor;
	}
}
}

inline void interpolateExponential(unsigned dimtime,unsigned dimstates,double tau,double* startpoint, double* endpoint,double** intmatrix)
{
	unsigned i,j;
	double timefactor;
	for(j=0;j<dimtime;j++){ ///Every Column
		timefactor=(((float)j)/((float)(dimtime-1)));
		for(i=0;i<dimstates;i++){ ///Every Row
			intmatrix[j][i]=startpoint[i]+(endpoint[i]-startpoint[i])*(1-std::exp(-timefactor*tau));
		}
	}

}
inline void interpolateRoot(unsigned dimtime,unsigned dimstates,double tau,double* startpoint, double* endpoint,double** intmatrix)
{
	unsigned i,j;
	double timefactor;
	for(j=0;j<dimtime;j++){ ///Every Column
		timefactor=(((float)j)/((float)(dimtime-1)));
		for(i=0;i<dimstates;i++){ ///Every Row
			intmatrix[j][i]=startpoint[i]+(endpoint[i]-startpoint[i])*std::sqrt(j/(dimtime));
		}
	}
}

inline void free(double** a){
	//printf("free matrix\n");
	std::free(a[0]);
	std::free(a);
}

inline void free(double* a){
	//std::printf("free vector\n");
	std::free(a);
}

inline void PoseStamped2Position(std::vector<double>& array, geometry_msgs::PoseStamped msgPose,double& rostimestamp){
	rostimestamp=msgPose.header.stamp.toSec();
	array[0]=msgPose.pose.position.x;
	array[1]=msgPose.pose.position.y;
	array[2]=msgPose.pose.position.z;
	array[3]=0;
	array[4]=0;
	array[5]=0;
	array[6]=0;
}

inline void PoseStamped2Array(std::vector<double>& array, geometry_msgs::PoseStamped msgPose,double& rostimestamp){
	rostimestamp=msgPose.header.stamp.toSec();
	array[0]=msgPose.pose.position.x;
	array[1]=msgPose.pose.position.y;
	array[2]=msgPose.pose.position.z;
	array[3]=msgPose.pose.orientation.x;
	array[4]=msgPose.pose.orientation.y;
	array[5]=msgPose.pose.orientation.z;
	array[6]=msgPose.pose.orientation.w;
}

void PoseStamped2State(std::vector<double>& state, const geometry_msgs::PoseStamped msgPose, double laststateupdatetimestamp, double& newstatetimestamp);



}





#endif

