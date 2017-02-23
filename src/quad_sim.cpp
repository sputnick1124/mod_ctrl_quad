#include <iostream>
#include "ros/ros.h"
#include <cmath>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <mod_ctrl_quad/ForcesConfig.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>

using namespace std;
using namespace Eigen;

class Quadrotor
{
    private:
        double Jx, Jy, Jz, m, g;
        VectorXd F = VectorXd(4);

        tf::TransformBroadcaster br;

        void wrench_callback(const geometry_msgs::Wrench::ConstPtr& msg)
        {
            double f, Tx, Ty, Tz;
            cout << msg->force.z <<endl;
            f = msg->force.z;
            Tx = msg->torque.x;
            Ty = msg->torque.y;
            Tz = msg->torque.z;
            F << f, Tx, Ty, Tz;
            set_control(f,Tx,Ty,Tz);
        };

        void set_control(double f, double Tx, double Ty, double Tz)
        {
//            F << f, Tx, Ty, Tz;
            F(0) = f;
            F(1) = Tx;
            F(2) = Ty;
            F(3) = Tz;
            cout << F <<endl;
        };

        ros::Publisher pose_pub;
        ros::Subscriber wrench_sub;
        ros::Rate loop_rate = ros::Rate(200);

//        tuple<double, double,double,double,double,double,double,double,double,double,double,double,double>
        tuple<double,VectorXd> quad_dynamics(double, VectorXd);

        void quad_run(double t, VectorXd x)
        {
            double t_n;
            VectorXd x_n(12);
            while(1)
            {
                tie(t_n,x_n) = quad_dynamics(t,x);
            }
        };


    public:
        Quadrotor(ros::NodeHandle nh)
        {
            VectorXd x0(12);
            vector<double> x0temp;
//            double x0[12];

            wrench_sub = nh.subscribe("quad_dynamics/quad_wrench",1,&Quadrotor::wrench_callback,this);
            pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/quad_dynamics/quad_pose",100);
            nh.getParam("/quad_dynamics/x0",x0temp);
            nh.getParam("/quad_dynamics/Jx",Jx);
            nh.getParam("/quad_dynamics/Jy",Jy);
            nh.getParam("/quad_dynamics/Jz",Jz);
            nh.getParam("/quad_dynamics/m",m);
            nh.getParam("/quad_dynamics/g",g);


            for (int i = 0; i < 12; i++)
            {
                x0(i)  = x0temp[i];
            }

            set_control(m*g,0,0,0.01);
            double t = ros::Time::now().toSec();
//            quad_dynamics(t,x0);
            quad_run(t,x0);
        }
};


tuple<double,VectorXd>
Quadrotor::quad_dynamics(
    double t, VectorXd x)
{
    VectorXd dx(12);
    double u,v,w,phi,theta,psi,p,q,r;

    u       = x(3);
    v       = x(4);
    w       = x(5);
    phi     = x(6);
    theta   = x(7);
    psi     = x(8);
    p       = x(9);
    q       = x(10);
    r       = x(11);

    dx(0) = cos(theta)*cos(psi)*u + sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)*v + cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)*w;
    dx(1) = cos(theta)*sin(psi)*u + sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)*v + cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)*w;
    dx(2) = sin(theta)*u - sin(phi)*cos(theta)*v - cos(phi)*cos(theta)*w;

    dx(3) = r*v - q*w - g*sin(theta);
    dx(4) = p*w - r*u + g*cos(theta)*sin(phi);
    dx(5) = q*u - p*v + g*cos(theta)*cos(phi) - F(0)/m;

    dx(6) = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    dx(7) = cos(phi)*q - sin(phi)*r;
    dx(8) = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;

    dx(9) = (Jy-Jz)/Jx*q*r + 1/Jx*F(1);
    dx(10) = (Jz-Jx)/Jy*p*r + 1/Jy*F(2);
    dx(11) = (Jx-Jy)/Jz*p*q + 1/Jz*F(3);

    geometry_msgs::PoseStamped msg;

    msg.header.stamp = ros::Time::now();

    msg.pose.position.x = x(0);
    msg.pose.position.y = x(1);
    msg.pose.position.z = x(2);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x(0),x(1),x(2)));
    tf::Quaternion quat = tf::createQuaternionFromRPY(phi,theta,psi);
    transform.setRotation(quat);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world","quad"));
    msg.header.frame_id = "world";
    msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(phi,theta,psi);
    pose_pub.publish(msg);
    loop_rate.sleep();
    if (ros::ok())
    {
        double t1 = ros::Time::now().toSec();
        double dt = t1 - t;

        return make_tuple(t1,x + dx*dt);
//        quad_dynamics(t1,xtemp);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quad_sim");
    ros::NodeHandle nh;

    Quadrotor quad(nh);

    ros::spin();

    return 0;
}
