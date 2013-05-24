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
 * @file odometry_markers_node.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 19, 2012
 *
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;
long id;
long id_odom;

void odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
	id_odom++;
	ros::Time current_time = ros::Time::now();

	visualization_msgs::Marker marker;

	marker.header.frame_id = "/world";
	marker.header.stamp = current_time;
	marker.ns = "odometry_marker";
	marker.id = id_odom;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose = msg->pose.pose;

	marker.scale.x = 0.1;
	marker.scale.y = 0.24;
	marker.scale.z = 0.02;
	marker.color.a = 0.8;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	marker_pub.publish(marker);
}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr & msg)
{
	id++;
	ros::Time current_time = ros::Time::now();

	visualization_msgs::Marker marker;
	visualization_msgs::Marker marker_lfw;
	visualization_msgs::Marker marker_rfw;

	tf::Quaternion q = tf::createQuaternionFromYaw(msg->theta);
	marker.header.frame_id = "/world";
	marker.header.stamp = current_time;
	marker.ns = "pose_marker";
	marker.id = id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	/*
	 marker.pose.position.x = 0; //msg->x;
	 marker.pose.position.y = 0; //msg->y;
	 marker.pose.position.z = 0;
	 marker.pose.orientation.x = 0; //q.x();
	 marker.pose.orientation.y = 0; //q.y();
	 marker.pose.orientation.z = 0; //q.z();
	 marker.pose.orientation.w = 1; //q.w();
	 */
	marker.pose.position.x = msg->x;
	marker.pose.position.y = msg->y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = q.x();
	marker.pose.orientation.y = q.y();
	marker.pose.orientation.z = q.z();
	marker.pose.orientation.w = q.w();

	marker.scale.x = 0.1;
	marker.scale.y = 0.24;
	marker.scale.z = 0.02;
	marker.color.a = 0.8;
	marker.color.r = 1.0;
	marker.color.g = 0.5; //(id % 256) / 255.0;
	marker.color.b = 0.0;

	marker_pub.publish(marker);
	/*
	 marker_lfw.header.frame_id = "/front_left_wheel_dir";
	 marker_lfw.header.stamp = current_time;
	 marker_lfw.ns = "/odometry_markers_left";
	 marker_lfw.id = id;
	 marker_lfw.type = visualization_msgs::Marker::CUBE;
	 marker_lfw.action = visualization_msgs::Marker::ADD;
	 marker_lfw.scale.x = 0.1;
	 marker_lfw.scale.y = 0.24;
	 marker_lfw.scale.z = 0.02;
	 marker_lfw.color.a = 0.8;
	 marker_lfw.color.r = 0.0;
	 marker_lfw.color.g = 0.0;
	 marker_lfw.color.b = 1.0;
	 marker_lfw.pose.position.z = -0.24;
	 marker_lfw.pose.position.y = 0.1;

	 marker_pub.publish(marker_lfw);
	 */
	marker_rfw.header.frame_id = "/front_right_wheel_dir";
	marker_rfw.header.stamp = current_time;
	marker_rfw.ns = "/pose_markers_right";
	marker_rfw.id = id;
	marker_rfw.type = visualization_msgs::Marker::CUBE;
	marker_rfw.action = visualization_msgs::Marker::ADD;
	marker_rfw.scale.x = 0.1;
	marker_rfw.scale.y = 0.24;
	marker_rfw.scale.z = 0.02;
	marker_rfw.color.a = 0.8;
	marker_rfw.color.r = 0.0;
	marker_rfw.color.g = 1.0;
	marker_rfw.color.b = 0.0;
	marker_rfw.pose.position.z = -0.24;
	marker_rfw.pose.position.y = -0.1;

	marker_pub.publish(marker_rfw);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_markers_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	id = 0;
	id_odom = 0;
	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose2D>("pose2d", 1, poseCallback);
	marker_pub = nh.advertise<visualization_msgs::Marker>(nh_priv.getNamespace() +  "/visualization_marker", 1);

	ros::spin();
}

