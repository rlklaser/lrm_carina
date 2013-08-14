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
 * @file controle_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Mar 29, 2013
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <lrm_msgs/Steering.h>
#include <lrm_msgs/Velocity.h>

#include <math.h>

using namespace message_filters;

ros::Publisher _controle_pub;

geometry_msgs::Twist twist_;
double _max_velocity;

void callback_vel(const lrm_msgs::Velocity::ConstPtr msgs) {
	twist_.linear.x = msgs->value * _max_velocity/100;
	_controle_pub.publish(twist_);
}

void callback_steer(const lrm_msgs::Steering::ConstPtr msgs) {
	twist_.angular.z = msgs->angle * M_PI / 180;
	_controle_pub.publish(twist_);
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "car_message_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	nh.param("max_velocity", _max_velocity, 5.0);

	_controle_pub = nh.advertise<geometry_msgs::Twist>(nh_priv.getNamespace() + "/cmd_vel", 1);
	ros::Subscriber sub_steer = nh.subscribe("steering_commands", 1, callback_steer);
	ros::Subscriber sub_vel = nh.subscribe("velocity_commands", 1, callback_vel);

	ROS_INFO_STREAM("car message node start spinning...");

	ros::spin();

	return 0;
}
