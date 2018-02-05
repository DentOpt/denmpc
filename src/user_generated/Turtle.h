/**
 * @file    Turtle.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Exported with Mathematica Code Generator by Jan Dentler
 */

#ifndef TURTLE_H_
#define TURTLE_H_

#include <Agent.h>
 #include <turtlesim/Pose.h>

/*******************************************************
 * Agent                                               * 
 *******************************************************/ 

class Turtle:public Agent{
    turtlesim::Pose subscriber0_old_msg_;
    turtlesim::Pose subscriber1_old_msg_;

public:
    Turtle(int id=0);
    Turtle(
        std::string pose,
        std::string desiredpose,
        std::string cmd_vel,
        double* init_x,
        double* init_xdes,
        double* init_u,
        double* init_udes,
        double* init_p,
        double* init_d,
        int id
    );
void setStateSubscriberRosTopicName(std::string rostopicname){
    ros_state_subscribers_[0]->shutdown();
    *ros_state_subscribers_[0]=ros_node_.subscribe<turtlesim::Pose>(rostopicname, 1,&Turtle::subStateCallback,this);
};
void subStateCallback(const turtlesim::Pose::ConstPtr& msg){
          std::vector<double>tmp(dim_x_,0);
        //  double dt=(msg->header.stamp.nsec-subscriber0_old_msg_.header.stamp.nsec)*1.0e-9;
        //  if(dt>0){

               tmp[0]=msg->x;
               tmp[1]=msg->y;
               tmp[2]=msg->theta;
               this->setState(tmp);
               subscriber0_old_msg_=*msg;
          // }
       };
    void setDesiredStateSubscriberRosTopicName(std::string rostopicname){
        ros_desired_state_subscribers_[0]->shutdown();
        *ros_desired_state_subscribers_[0]=ros_node_.subscribe<turtlesim::Pose>(rostopicname, 1,&Turtle::subDesiredStateCallback,this);
    };
    void subDesiredStateCallback(const turtlesim::Pose::ConstPtr & msg){
        std::vector<double> tmp(dim_x_,0);
      //  double dt=(msg->header.stamp.nsec-subscriber1_old_msg_.header.stamp.nsec)*1.0e-9;
      //  if(dt>0){

               tmp[0]=msg->x;
               tmp[1]=msg->y;
               tmp[2]=msg->theta;
            this->setDesiredState(tmp);
            subscriber1_old_msg_=*msg;
      //  }
    }

    void setPublisherRosTopicName(std::string rostopicname){
        ros_publishers_[0]->shutdown();
        *ros_publishers_[0]=ros_node_.advertise<geometry_msgs::Twist>(rostopicname, 1);
    };
    void rosPublishActuation(){
        geometry_msgs::Twist msg;
        msg.linear.x = u_[0];
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = u_[1];
        ros_publishers_[0]->publish(msg);
    }
    void f(double  *out,double t, double *x, double *u, double *d, double *p);
    void dfdxlambda(double  *out,double t, double *x, double *u, double *d, double *p, double *lambda);
    void dfdulambda(double  *out,double t, double *x, double *u, double *d, double *p, double *lambda);
    void l(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes);
    void dldx(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes);
    void dldu(double  *out,double t, double *x, double *u, double *p, double *xdes, double *udes);
    void v(double  *out,double t, double *x, double *p, double *xdes);
    void dvdx(double  *out,double t, double *x, double *p, double *xdes);
    void ci(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes);
    void dcidxmui(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui);
    void dcidumui(double *out, double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui);
    void cia(double  *out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *slack);
    void dciadxmui(double  *out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui, double *slack);
    void dciadumui(double  *out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mu, double *slack);
    void dciadamui(double  *out,double t, double *x, double *u, double *p, double *pc,  double *xdes, double *udes, double *mui, double *slack);
};
#endif
