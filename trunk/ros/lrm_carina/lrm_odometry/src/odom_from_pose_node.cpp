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
 * @file odom_from_pose_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jun 4, 2013
 *
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher odom_pub;
tf::TransformBroadcaster* odom_broadcaster;
nav_msgs::Odometry odom_msgs;

void poseCallback(const geometry_msgs::PoseWithCovarianceStampedPtr msg) {

	//odom_msgs.header.frame_id = msg->header.frame_id;
	//odom_msgs.header.stamp = msg->header.stamp;

	odom_msgs.header = msg->header;
	odom_msgs.header.frame_id = "/odom";
	odom_msgs.child_frame_id = "/base_footprint";
	odom_msgs.pose = msg->pose;
	odom_msgs.pose.pose.position.z = 0;

	odom_pub.publish(odom_msgs);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "odom_from_pose_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	odom_broadcaster = new tf::TransformBroadcaster();

	odom_pub = nh.advertise<nav_msgs::Odometry>(nh_priv.getNamespace() + "/odom", 1);
	ros::Subscriber pose_sub = nh.subscribe("pose", 1, poseCallback);

	ros::spin();

	delete odom_broadcaster;

	return 0;
}

