/**
 * @file    Ardrone_20170227.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Exported with Mathematica Code Generator by Jan Dentler
 */

#ifndef ARDRONE_20170227_H_
#define ARDRONE_20170227_H_

#include <Agent.h>

/*******************************************************
 * Agent                                               * 
 *******************************************************/ 

class Ardrone_20170227:public Agent{
    geometry_msgs::PoseStamped subscriber0_old_msg_;
    geometry_msgs::PoseStamped subscriber1_old_msg_;

public:
    Ardrone_20170227(int id=0);
    Ardrone_20170227(
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
    *ros_state_subscribers_[0]=ros_node_.subscribe<geometry_msgs::PoseStamped>(rostopicname, 1,&Ardrone_20170227::subStateCallback,this);
};
void subStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
          std::vector<double>tmp(dim_x_,0);
          double dt=(msg->header.stamp.nsec-subscriber0_old_msg_.header.stamp.nsec)*1.0e-9;
          if(dt>0){
               tf::Quaternion quat(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
               tf::Quaternion quat_old(subscriber0_old_msg_.pose.orientation.x,subscriber0_old_msg_.pose.orientation.y,subscriber0_old_msg_.pose.orientation.z,subscriber0_old_msg_.pose.orientation.w);
               double tmp1,tmp2,yaw,yaw_old;
               tf::Matrix3x3(quat).getEulerYPR(yaw,tmp1,tmp2);
               tf::Matrix3x3(quat_old).getEulerYPR(yaw_old,tmp1,tmp2);

               double dx=(msg->pose.position.x-subscriber0_old_msg_.pose.position.x)/dt; 
               double dy=(msg->pose.position.y-subscriber0_old_msg_.pose.position.y)/dt;
               double dz=(msg->pose.position.z-subscriber0_old_msg_.pose.position.z)/dt;
               double dyaw=(yaw-yaw_old)/dt;

               tmp[0]=msg->pose.position.x;
               tmp[1]=msg->pose.position.y;
               tmp[2]=msg->pose.position.z;
               tmp[3]=cos(yaw);
               tmp[4]=sin(yaw);
               tmp[5]=dx*tmp[3]+dy*tmp[4];
               tmp[6]=-dx*tmp[4]+dy*tmp[3];
               tmp[7]=dz;
               tmp[8]=dyaw;
               this->setState(tmp);
               subscriber0_old_msg_=*msg;
           }
       };
    void setDesiredStateSubscriberRosTopicName(std::string rostopicname){
        ros_desired_state_subscribers_[0]->shutdown();
        *ros_desired_state_subscribers_[0]=ros_node_.subscribe<geometry_msgs::PoseStamped>(rostopicname, 1,&Ardrone_20170227::subDesiredStateCallback,this);
    };
    void subDesiredStateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        std::vector<double> tmp(dim_x_,0);
        double dt=(msg->header.stamp.nsec-subscriber1_old_msg_.header.stamp.nsec)*1.0e-9;
        if(dt>0){
            tf::Quaternion quat(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
            tf::Quaternion quat_old(subscriber1_old_msg_.pose.orientation.x,subscriber1_old_msg_.pose.orientation.y,subscriber1_old_msg_.pose.orientation.z,subscriber1_old_msg_.pose.orientation.w);
            double tmp1,tmp2,yaw,yaw_old;
            tf::Matrix3x3(quat).getEulerYPR(yaw,tmp1,tmp2);
            tf::Matrix3x3(quat_old).getEulerYPR(yaw,tmp1,tmp2);

            double dx=(msg->pose.position.x-subscriber1_old_msg_.pose.position.x)/dt;
            double dy=(msg->pose.position.y-subscriber1_old_msg_.pose.position.y)/dt;
            double dz=(msg->pose.position.z-subscriber1_old_msg_.pose.position.z)/dt;
            double dyaw=(yaw-yaw_old)/dt;

            tmp[0] = msg->pose.position.x;
            tmp[1] = msg->pose.position.y;
            tmp[2] = msg->pose.position.z;
            tmp[3]=cos(yaw);
            tmp[4]=sin(yaw);
            tmp[5]=dx*tmp[3]+dy*tmp[4];
            tmp[6]=-dx*tmp[4]+dy*tmp[3];
            tmp[7]=dz;
            tmp[8]=dyaw;
            this->setDesiredState(tmp);
            subscriber1_old_msg_=*msg;
        }
    }

    void setPublisherRosTopicName(std::string rostopicname){
        ros_publishers_[0]->shutdown();
        *ros_publishers_[0]=ros_node_.advertise<geometry_msgs::Twist>(rostopicname, 1);
    };
    void rosPublishActuation(){
        geometry_msgs::Twist msg;
        msg.linear.x = u_[0];
        msg.linear.y = u_[1];
        msg.linear.z = u_[2];
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = u_[3];
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
