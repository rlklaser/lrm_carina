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
 * @file constant_twist_publisher.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Dec 11, 2013
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "constant_twist_publisher");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	geometry_msgs::Twist msg;

	double rate;
	double linear_x;
	double angular_z;
	//std::string frame_id;

	nh_priv.param("rate", rate, 10.0);
	//nh_priv.param("frame_id", frame_id, std::string("base_link"));
	nh_priv.param("linear_x", linear_x, 0.0);
	nh_priv.param("angular_z", angular_z, 0.0);

	ros::Publisher twist_pub = nh_priv.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	ROS_INFO_STREAM("constant twist publisher started");

	msg.linear.x = linear_x;
	msg.angular.z = angular_z;

	ros::Rate hz(rate);

	while(ros::ok()) {
		twist_pub.publish(msg);
		ros::spinOnce();
		hz.sleep();
	}
}
