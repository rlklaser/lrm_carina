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
 * @file vel_booster_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Aug 24, 2013
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher _vel_pub;
double _max_velocity;
double _max_velocity_boost;

void callbackTwist(const geometry_msgs::Twist::ConstPtr msg) {
	geometry_msgs::Twist twist;

	//twist.header = msg->header;
	twist.angular = msg->angular;
	twist.linear.x = msg->linear.x * (_max_velocity_boost/_max_velocity);

	_vel_pub.publish(twist);
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "val_booster_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	nh.param("max_velocity", _max_velocity, 0.4);
	nh.param("max_velocity_boost", _max_velocity_boost, 5.0);

	ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, callbackTwist);
	_vel_pub = nh.advertise<geometry_msgs::Twist>(nh_priv.getNamespace() + "/cmd_vel", 1);

	ROS_INFO_STREAM("vel booster node start spinning...");

	ros::spin();

	return 0;
}
