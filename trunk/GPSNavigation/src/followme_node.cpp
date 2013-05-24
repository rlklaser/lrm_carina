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
 * @file followme_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 10, 2013
 *
 */


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include "GPSNavigation/Goal.h"


struct goal {
	double x;
	double y;
	double theta;
	bool visited;
};


geometry_msgs::PoseStamped goal;
nav_msgs::Odometry odom;
ros::Publisher goal_pub;
ros::Timer timeout_timer;

double pose_x;
double pose_y;
bool first_time;
double dist_factor;
double timeout;
bool reached;

std::string frame_id;


void publishNextGoal()
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

	//starts algorithm on first odometry message
	if(first_time) {
		first_time = false;
		pose_x=odom.pose.pose.position.x;
		pose_y=odom.pose.pose.position.y;
		//publishFromKernel();
	}
}


void reachedReceived(const GPSNavigation::Goal::ConstPtr& msg)
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


	publishNextGoal();

}

void timeoutCallback(const ros::TimerEvent& t)
{
	timeout_timer.stop();
	ROS_WARN("timeout... new position");
	publishNextGoal();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "followme_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    pn.param("dist_factor", dist_factor, KERNEL_DIST_FACTOR);
    pn.param("timeout", timeout, 30.0);
    pn.param<std::string>("frame_id", frame_id, "/base_link");


    goal_pub = n.advertise<geometry_msgs::PoseStamped>(n.getNamespace() + "/move_base_simple/goal", 5, true);
    timeout_timer = n.createTimer(ros::Duration(timeout), timeoutCallback);


    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(n.getNamespace() + "/odom", 20, odomReceived);
    ros::Subscriber signal_sub = n.subscribe<lrm_rbb_grupo_c::Signal>(n.getNamespace() + "/signal", 2, signalReceived);
    ros::Subscriber reached_sub = n.subscribe<lrm_rbb_grupo_c::Goal>(n.getNamespace() + "/move_base_simple/reached", 2, reachedReceived);

    pose_x = 0;
    pose_y = 0;
    reached = false;

    //starts on first odometry msg
    first_time = true;

    ROS_INFO_STREAM(
    		"Follow-Me navigation started" << std::endl <<
    		"\ns=" << n.getNamespace() << " ~ns=" << pn.getNamespace() << std::endl <<
    		"\tdist_factor=" << dist_factor << std::endl <<
    		"\ttimeout=" << timeout << std::endl <<
    		"\tframe_id=" << frame_id << std::endl
    		);

    ros::spin();
}
