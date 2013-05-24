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
 * @file nav_message_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Apr 23, 2013
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <lrm_msgs/Steering.h>
#include <lrm_msgs/Velocity.h>

#include <math.h>

ros::Publisher steer_pub;
ros::Publisher vel_pub;

void callbackTwist(const geometry_msgs::Twist::ConstPtr msg) {
	lrm_msgs::Steering steer_msg;
	lrm_msgs::Velocity vel_msg;

	ros::Time current_timestamp = ros::Time::now();

	steer_msg.header.stamp = current_timestamp;
	vel_msg.header.stamp = current_timestamp;

	steer_msg.angle = msg->angular.z / M_PI * 180;
	vel_msg.value = msg->linear.x;

	steer_pub.publish(steer_msg);
	vel_pub.publish(vel_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "nav_message_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::Subscriber cmdvel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, callbackTwist);
	steer_pub = nh.advertise<lrm_msgs::Steering>(nh_priv.getNamespace() + "/steering_commands", 1);
	vel_pub = nh.advertise<lrm_msgs::Velocity>(nh_priv.getNamespace() + "/velocity_commands", 1);

	ROS_INFO_STREAM("nav message node start spinning...");

	ros::spin();
}
