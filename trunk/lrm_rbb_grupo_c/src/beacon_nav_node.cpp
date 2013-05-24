/*
 *  Copyright (C) 2012, Laboratorio de Robotica Movel - ICMC/USP
 *  Rafael Luiz Klaser <rlklaser@gmail.com>
 *  http://lrm.icmc.usp.br
 *
 *  Apoio FAPESP: 2012/04555-4
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file beacon_nav_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 28, 2012
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <lrm_rbb_grupo_c/Signal.h>
#include <lrm_rbb_grupo_c/Goal.h>


struct goal {
	double x;
	double y;
	double theta;
	bool visited;
};

#define KERNEL_DIST_FACTOR	4.0
#define KERNEL_SIZE 8

//circular kernel
struct goal visit_kernel[] = {
/*0*/	{+1, +0, 0, 		false},
/*1*/	{+1, -1, -M_PI/4, 	false},
/*2*/	{+0, -1, -M_PI/2, 	false},
/*3*/	{-1, -1, -3*M_PI/3, false},
/*4*/	{-1, +0, -M_PI, 	false},
/*5*/	{-1, +1, 3*M_PI/3, 	false},
/*6*/	{+0, +1, M_PI/2, 	false},
/*7*/	{+1, +1, M_PI/4, 	false}
};
/*7*/	//{+3, +3, M_PI/4, 	false}

#if 0
//spiral kernel
struct goal visit_kernel[] = {
/*0*/	{+1, +0, 0, 		false},
/*1*/	{+1, -1, -M_PI/4, 	false},
/*2*/	{+0, -1, -M_PI/2, 	false},
/*3*/	{-1, -1, -3*M_PI/3, false},
/*4*/	{-1, +0, -M_PI, 	false},
/*5*/	{-1, +1, 3*M_PI/3, 	false},
/*6*/	{+0, +1, M_PI/2, 	false},
/*7*/	{+1, +1, M_PI/4, 	false}
};
#endif

int kernel_step;

geometry_msgs::PoseStamped goal;
nav_msgs::Odometry odom;
ros::Publisher goal_pub;
ros::Timer timeout_timer;
lrm_rbb_grupo_c::Signal curr_signal;
lrm_rbb_grupo_c::Signal best_signal;

double pose_x;
double pose_y;
bool first_time;
double dist_factor;
double timeout;
bool reached;

std::string frame_id;

void publishFromKernel()
{
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = frame_id;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(visit_kernel[kernel_step].theta);
    goal.pose.position.x = pose_x + (visit_kernel[kernel_step].x * dist_factor);
    goal.pose.position.y = pose_y + (visit_kernel[kernel_step].y * dist_factor);

    kernel_step++;
    kernel_step = kernel_step % KERNEL_SIZE;

    goal_pub.publish(goal);
    std::cout << "New goal (" << pose_x << ", " << pose_y << ")" << std::endl;
}

void publishFromSignal()
{
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = frame_id;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(best_signal.pose.theta);
    goal.pose.position.x = best_signal.pose.x;
    goal.pose.position.y = best_signal.pose.y;

    goal_pub.publish(goal);

    //TODO: period based on max velocity and distance
    //timeout_timer.setPeriod()
    timeout_timer.start();
    //kernel_step = 0;

    std::cout <<
    		"Search best signal (" <<
    		best_signal.pose.x <<
    		", " <<
    		best_signal.pose.y <<
    		") timeout triggered..." <<
    		std::endl;
}

void odomReceived(const nav_msgs::Odometry::ConstPtr& msg)
{
	odom = *msg;
    //std::cout << "odom" << std::endl;

	//starts algorithm on first odometry message
	if(first_time) {
		first_time = false;
		pose_x=odom.pose.pose.position.x;
		pose_y=odom.pose.pose.position.y;
		publishFromKernel();
	}
}

void signalReceived(const lrm_rbb_grupo_c::Signal::ConstPtr& msg)
{
	//build signal memory, or, just store better position
	curr_signal = *msg;

	curr_signal.pose.x = odom.pose.pose.position.x;
	curr_signal.pose.y = odom.pose.pose.position.y;
	curr_signal.pose.theta = tf::getYaw(odom.pose.pose.orientation);

	if(curr_signal.rssi>best_signal.rssi) {
		best_signal = curr_signal;
		//ROS_INFO_STREAM("signal improoved:" << best_signal.rssi);
	} else {
		//abort ???
	}

	//double e = (curr_signal.rssi + 28 - 20*log10(2401))/30;
	//double est_dist = 1/pow(10, e);
	//std::cout << "d:" << est_dist << "m" << std::endl;

}

void reachedReceived(const lrm_rbb_grupo_c::Goal::ConstPtr& msg)
{
	timeout_timer.stop();

	pose_x=odom.pose.pose.position.x;
	pose_y=odom.pose.pose.position.y;

	//search memory for a better signal, else goto next kernel position
	//std::cout << "REACHED (" << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ")" << std::endl;

	pose_x=msg->pose.x;
	pose_y=msg->pose.y;

	double area = fabs((pose_x-best_signal.pose.x)*(pose_y-best_signal.pose.y));

	std::cout << "REACHED (" << pose_x << ", " << pose_y << ") a:" << area << std::endl;

	if(curr_signal.rssi/2 + curr_signal.dBm > 0.01) {
		reached = true;
		std::cout << "Tadaaaaaa!!!" << std::endl;
		return;
	}

	//se esta dentro da area X do melhor sinal, busca, senao vai ate o melhor sinal

	if(area<0.25 /*|| kernel_step!=0*/) {
		publishFromKernel();
	}
	else {
		publishFromSignal();
	}


}

void timeoutCallback(const ros::TimerEvent& t)
{
	timeout_timer.stop();
	ROS_WARN("timeout... new position");
	publishFromKernel();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "beacon_nav_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    pn.param("dist_factor", dist_factor, KERNEL_DIST_FACTOR);
    pn.param("timeout", timeout, 30.0);
    pn.param<std::string>("frame_id", frame_id, "/base_link");


    goal_pub = n.advertise<geometry_msgs::PoseStamped>(n.getNamespace() + "/move_base_simple/goal", 5, true);
    timeout_timer = n.createTimer(ros::Duration(timeout), timeoutCallback);

    //std::cout << "ZZZ:" << n.getNamespace() << std::endl;

    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(n.getNamespace() + "/odom", 20, odomReceived);
    ros::Subscriber signal_sub = n.subscribe<lrm_rbb_grupo_c::Signal>(n.getNamespace() + "/signal", 2, signalReceived);
    ros::Subscriber reached_sub = n.subscribe<lrm_rbb_grupo_c::Goal>(n.getNamespace() + "/move_base_simple/reached", 2, reachedReceived);

    kernel_step = 0;
    pose_x = 0;
    pose_y = 0;
    best_signal.rssi = -99999;
    reached = false;

    //starts on first odometry msg
    first_time = true;
    //publishFromKernel();

    ROS_INFO_STREAM(
    		"Beacon navigation started" << std::endl <<
    		"\ns=" << n.getNamespace() << " ~ns=" << pn.getNamespace() << std::endl <<
    		"\tdist_factor=" << dist_factor << std::endl <<
    		"\ttimeout=" << timeout << std::endl <<
    		"\tframe_id=" << frame_id << std::endl
    		);

    ros::spin();
}
