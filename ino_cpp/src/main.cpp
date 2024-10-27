#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose2D.h>
#include "geometry_msgs/Twist.h" 
#include <nav_msgs/Odometry.h>
#include"MPC.h"
#include<math.h>
#include<iostream>
using namespace std;
int i =0;
int max_iter = 1000; 
unsigned int SampleTime =1;

geometry_msgs::Pose2D vel;
nav_msgs::Odometry odometry;
geometry_msgs::Pose2D pos;
//geometry_msgs::Twist vel;
void callback(const geometry_msgs::Pose2D &xytheta) {
	pos = xytheta;
	ROS_INFO("I got a 2d pose!");
}


void callback_odom(const nav_msgs::Odometry &odom) {
	odometry = odom;
	ROS_INFO("I got an odometry!");
}


int main(int argc, char **argv) {
	double Q[3]={25, 25 ,1}; 
	double R[2]= {1, 1};



	ros::init(argc, argv, "pi_node");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<geometry_msgs::Pose2D>("desired_velocity", 5);
	
	ros::Subscriber sub = n.subscribe("actual_position", 5, callback);


	ros::Subscriber odom = n.subscribe("odom", 5, callback_odom);



	// Running at 1Hz
	ros::Rate loop_rate(1);
	ros::spinOnce();


//generate reference
double** Xref = cercleRef(max_iter, SampleTime, 0.002, 3, 1.5, 0, 0);
//double** Xref = rightRef(max_iter, SampleTime, 0);
swarm my_swarm;

 while(i <= max_iter){

	while (ros::ok()) {		

                //double X[3] = {0, 0, 0};
                double X[3] = {odometry.pose.pose.position.x, odometry.pose.pose.position.y, 2*asin(odometry.pose.pose.orientation.z)};
                double* Vopt= MPC(Xref, X, 20, 150, Q, R, SampleTime ,i, 0.0325, 0.445, my_swarm);                
		//vel.x = Vopt[0];
                //vel.y = Vopt[1];
cout << "2D position : " << X[0] << "  " << X[1] << "  " << X[2]/M_PI*180 << endl;
		vel.linear.x = Vopt[0];
		vel.linear.y = 0;
		vel.angular.z = Vopt[1];

		pub.publish(vel);
		ros::spinOnce();
		loop_rate.sleep();
		i++;
	}
}

	return 0;
}
