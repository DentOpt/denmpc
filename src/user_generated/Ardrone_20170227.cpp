/**
 * @file    Ardrone_20170227.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Exported with Mathematica Code Generator by Jan Dentler
 */

 #include <Ardrone_20170227.h>

/*******************************************************
 * Constructor                                         * 
 *******************************************************/ 
Ardrone_20170227::Ardrone_20170227(int id):Agent(id){
    //Set initial values
    dim_x_=9;
    dim_xdes_=9;
    dim_u_=4;
    dim_udes_=4;
    dim_y_=0;
    dim_ydes_=0;
    dim_p_=19;
    dim_d_=0;
    dim_l_=1;
    dim_v_=1;
    dim_eq_=0;
    dim_ineq_=4;
    //Allocate vectors
    x_     =defvector(dim_x_);
    xdes_  =defvector(dim_xdes_);
    u_     =defvector(dim_u_);
    udes_  =defvector(dim_udes_);
    y_     =defvector(dim_y_);
    ydes_  =defvector(dim_ydes_);
    d_     =defvector(dim_d_);
    p_     =defvector(dim_p_);
    x_init_    =defvector(dim_x_);
    xdes_init_ =defvector(dim_xdes_);
    u_init_    =defvector(dim_u_);
    udes_init_ =defvector(dim_udes_);
    y_init_    =defvector(dim_y_);
    ydes_init_ =defvector(dim_ydes_);
    d_init_    =defvector(dim_d_);
    p_init_    =defvector(dim_p_);

    //Creating control publisher
    ros::Publisher* pub0= new ros::Publisher();
    //Starting Advertising
    *pub0=ros_node_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    //Adding publisher to array
    ros_publishers_.push_back(pub0);
    
    //Creating subscriber
    ros::Subscriber* sub0= new ros::Subscriber();
    //Starting subscription
    *sub0=ros_node_.subscribe("pose", 1, &Ardrone_20170227::subStateCallback,this);
    //Adding subscriber to array
    ros_state_subscribers_.push_back(sub0);
    //Creating subscriber
    ros::Subscriber* sub1= new ros::Subscriber();
    //Starting subscription
    *sub1=ros_node_.subscribe("desiredpose", 1, &Ardrone_20170227::subDesiredStateCallback,this);
    //Adding subscriber to array
    ros_desired_state_subscribers_.push_back(sub1);
}
Ardrone_20170227::Ardrone_20170227(
    std::string state_subscriber_topic,
    std::string desired_state_subscriber_topic,
    std::string control_publish_topic,
    double* init_x=NULL,
    double* init_xdes=NULL,
    double* init_u=NULL,
    double* init_udes=NULL,
    double* init_p=NULL,
    double* init_d=NULL,
    int id=0
):Ardrone_20170227(id){
    if(init_x!=NULL){this->setInitialState (init_x);};
    if(init_xdes!=NULL){this->setInitialDesiredState (init_xdes);};
    if(init_u!=NULL){this->setInitialControl (init_u);};
    if(init_udes!=NULL){this->setInitialDesiredControl(init_udes);};
    if(init_p!=NULL){this->setInitialParameter (init_p);};
    this->reset2initialstate();
    this->setStateSubscriberRosTopicName(state_subscriber_topic);
    this->setDesiredStateSubscriberRosTopicName(desired_state_subscriber_topic);
    this->setPublisherRosTopicName(control_publish_topic);
};

/******************************************************** 
 * System dynamics                                      * 
 ********************************************************/ 
void Ardrone_20170227::f(double  *out,double t, double *x, double *u, double *d, double *p){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::f(...)"<<endl;
#endif
    out[0] = x[3]*x[5] - x[4]*x[6];
    out[1] = x[4]*x[5] + x[3]*x[6];
    out[2] = x[7];
    out[3] = -x[4]*x[8];
    out[4] = x[3]*x[8];
    out[5] = p[1]*u[0] + p[0]*x[5];
    out[6] = p[1]*u[1] + p[0]*x[6];
    out[7] = u[2]*p[3] + p[2]*x[7];
    out[8] = p[5]*u[3] + p[4]*x[8];
}
void Ardrone_20170227::dfdxlambda(double  *out,double t, double *x, double *u, double *d, double *p, double *lambda){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::dfdx(...)"<<endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = lambda[0]*x[5] + lambda[1]*x[6] + lambda[4]*x[8];
    out[4] = lambda[1]*x[5] - lambda[0]*x[6] - lambda[3]*x[8];
    out[5] = x[3]*lambda[0] + x[4]*lambda[1] + lambda[5]*p[0];
    out[6] = -x[4]*lambda[0] + x[3]*lambda[1] + lambda[6]*p[0];
    out[7] = lambda[2] + lambda[7]*p[2];
    out[8] = -x[4]*lambda[3] + x[3]*lambda[4] + lambda[8]*p[4];
}
void Ardrone_20170227::dfdulambda(double  *out,double t, double *x, double *u, double *d, double *p, double *lambda){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::dfdu(...)"<<endl;
#endif
    out[0] = lambda[5]*p[1];
    out[1] = lambda[6]*p[1];
    out[2] = lambda[7]*p[3];
    out[3] = lambda[8]*p[5];
}

/******************************************************** 
 * Stage costs                                          * 
 ********************************************************/ 
void Ardrone_20170227::l(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::l(...)"<<endl;
#endif
    out[0] = p[9]*pow(-x[3] + xdes[3],2.) + p[9]*pow(-x[4] + xdes[4],2.) + p[15]*pow(-u[0] + udes[0],2.) + p[15]*pow(-u[1] + udes[1],2.) + p[18]*pow(-u[3] + udes[\
3],2.) + p[17]*pow(-u[2] + udes[2],2.) + p[11]*pow(-x[5] + xdes[5],2.) + p[11]*pow(-x[6] + xdes[6],2.) + p[14]*pow(-x[8] + xdes[8],2.) + p[13]*pow(-x[7] + xdes[7\
],2.) + p[6]*pow(-x[0] + xdes[0],2.) + p[6]*pow(-x[1] + xdes[1],2.) + p[8]*pow(-x[2] + xdes[2],2.);
}
void Ardrone_20170227::dldx(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::dldx(...)"<<endl;
#endif
    out[0] = -2.*p[6]*(-x[0] + xdes[0]);
    out[1] = -2.*p[6]*(-x[1] + xdes[1]);
    out[2] = -2.*p[8]*(-x[2] + xdes[2]);
    out[3] = -2.*(-x[3] + xdes[3])*p[9];
    out[4] = -2.*(-x[4] + xdes[4])*p[9];
    out[5] = -2.*p[11]*(-x[5] + xdes[5]);
    out[6] = -2.*p[11]*(-x[6] + xdes[6]);
    out[7] = -2.*p[13]*(-x[7] + xdes[7]);
    out[8] = -2.*p[14]*(-x[8] + xdes[8]);
}
void Ardrone_20170227::dldu(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::dldu(...)"<<endl;
#endif
    out[0] = -2.*p[15]*(-u[0] + udes[0]);
    out[1] = -2.*p[15]*(-u[1] + udes[1]);
    out[2] = -2.*p[17]*(-u[2] + udes[2]);
    out[3] = -2.*p[18]*(-u[3] + udes[3]);
}

/******************************************************** 
 * Final costs                                          * 
 ********************************************************/ 
void Ardrone_20170227::v(double  *out,double t, double *x, double *p, double *xdes){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::v(...)"<<endl;
#endif
    out[0] = 0;
}
void Ardrone_20170227::dvdx(double  *out,double t, double *x, double *p, double *xdes){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::dvdx(...)"<<endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    out[4] = 0;
    out[5] = 0;
    out[6] = 0;
    out[7] = 0;
    out[8] = 0;
}

/******************************************************** 
 * Inequality constraints                               * 
 ********************************************************/ 
void Ardrone_20170227::ci(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::ci(...)"<<endl;
#endif
    out[0] = -0.5 + 0.5*pow(u[0],2.);
    out[1] = -0.5 + 0.5*pow(u[1],2.);
    out[2] = -0.5 + 0.5*pow(u[2],2.);
    out[3] = -0.5 + 0.5*pow(u[3],2.);
}
void Ardrone_20170227::dcidxmui(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::dcidxmui(...)"<<endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    out[4] = 0;
    out[5] = 0;
    out[6] = 0;
    out[7] = 0;
    out[8] = 0;
}
void Ardrone_20170227::dcidumui(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::dcidumui(...)"<<endl;
#endif
    out[0] = mui[0]*u[0];
    out[1] = mui[1]*u[1];
    out[2] = mui[2]*u[2];
    out[3] = mui[3]*u[3];
}
void Ardrone_20170227::cia(double  *out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *slack){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::cia(...)"<<endl;
#endif
    out[0] = -0.5 + pow(slack[0],2.) + 0.5*pow(u[0],2.);
    out[1] = -0.5 + pow(slack[1],2.) + 0.5*pow(u[1],2.);
    out[2] = -0.5 + pow(slack[2],2.) + 0.5*pow(u[2],2.);
    out[3] = -0.5 + pow(slack[3],2.) + 0.5*pow(u[3],2.);
}
void Ardrone_20170227::dciadxmui(double  *out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui, double *slack){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::dciadxmui(...)"<<endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    out[4] = 0;
    out[5] = 0;
    out[6] = 0;
    out[7] = 0;
    out[8] = 0;
}
void Ardrone_20170227::dciadumui(double  *out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui, double *slack){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::dciadumui(...)"<<endl;
#endif
    out[0] = mui[0]*u[0];
    out[1] = mui[1]*u[1];
    out[2] = mui[2]*u[2];
    out[3] = mui[3]*u[3];
}
void Ardrone_20170227::dciadamui(double  *out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui, double *slack){
#ifdef DEBUG_FUNCTION_TRACE
cout<<"exec Ardrone_20170227::dciadamui(...)"<<endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
}
