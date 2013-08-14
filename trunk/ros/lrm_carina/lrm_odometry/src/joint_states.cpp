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
 * @file joint_states.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Dec 19, 2012
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "joint_states_node");
	ros::NodeHandle nh;

	ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
	sensor_msgs::JointState joint_state;

	ros::Rate r(30);
	while(ros::ok()) {
		ros::spinOnce();

		joint_state.header.stamp = ros::Time::now();

		r.sleep();
	}
}
