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
 * @file imu_yaw_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jun 28, 2013
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <angles/angles.h>

void imuCallback(sensor_msgs::Imu::ConstPtr msg) {

	ROS_INFO_STREAM("yaw:" << angles::to_degrees(tf::getYaw(msg->orientation)));
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "imu_yaw_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	///lse_xsens_mti/xsens/imu/data
	ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu_data", 1, imuCallback);

	ros::spin();
}
