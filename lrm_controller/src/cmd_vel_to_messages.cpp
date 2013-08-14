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
 * @file cmd_vel_to_messages.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 23, 2013
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include <lrm_msgs/Throttle.h>
#include <lrm_msgs/Steering.h>
#include <angles/angles.h>

geometry_msgs::Twist base_vel_msg_;
double previous_linear_vel = 0.0;

void commandCallback(const geometry_msgs::TwistConstPtr& msg) {
	base_vel_msg_ = *msg;
	previous_linear_vel = base_vel_msg_.linear.x;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "cmd_vel_to_messages");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::Subscriber cmd_sub_;

	ros::Publisher throttle_pub;
	ros::Publisher steering_pub;
	ros::Rate rate(20);

	lrm_msgs::Throttle throttle;
	lrm_msgs::Steering steer;

	geometry_msgs::Twist cmd_vel_;
	float pose_x, pose_y;

	throttle_pub = nh.advertise<lrm_msgs::Throttle>(nh_priv.getNamespace() + "/throttle_commands", 1);
	steering_pub = nh.advertise<lrm_msgs::Steering>(nh_priv.getNamespace() + "/steering_commands", 1);
	cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &commandCallback);

	ROS_INFO("cmd_vel_to_messages node loaded!");

	while (nh.ok()) {

		throttle.value = std::max(std::min((int)(base_vel_msg_.linear.x * 100), 100), -100);
		//steer.angle = 57.3 * base_vel_msg_.angular.z; // radianos p/ graus
		steer.angle = angles::to_degrees(base_vel_msg_.angular.z); // radianos p/ graus

		throttle_pub.publish(throttle);
		steering_pub.publish(steer);

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
