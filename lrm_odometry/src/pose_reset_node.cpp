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
 * @file pose_reset_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Apr 25, 2013
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <lrm_odometry/SetPose.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

ros::ServiceClient client;
ros::Timer timer;
geometry_msgs::Pose2D pose;
boost::mutex mutex;

void poseCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg) {
	boost::unique_lock<boost::mutex> scoped_lock(mutex);

	//ROS_INFO_STREAM("odometry pose reset:" << msg->header.frame_id);

	pose.x = msg->pose.pose.position.x;
	pose.y = msg->pose.pose.position.y;
	pose.theta = tf::getYaw(msg->pose.pose.orientation);
}

void odomCallback(nav_msgs::Odometry::ConstPtr msg) {
	boost::unique_lock<boost::mutex> scoped_lock(mutex);

	//ROS_INFO_STREAM("odometry pose reset:" << msg->header.frame_id);

	pose.x = msg->pose.pose.position.x;
	pose.y = msg->pose.pose.position.y;
	pose.theta = tf::getYaw(msg->pose.pose.orientation);
}

void timerCallback(const ros::TimerEvent& e) {
	boost::unique_lock<boost::mutex> scoped_lock(mutex);

	lrm_odometry::SetPose srv;

	srv.request.pose.x = pose.x;
	srv.request.pose.y = pose.y;
	srv.request.pose.theta = pose.theta;

	if (client.call(srv)) {
		ROS_WARN_STREAM("odometry pose reset (x,y,tetha) (" << pose.x << ", " << pose.y << ", " << pose.theta << ")");
	} else {
		ROS_ERROR_STREAM("odometry set pose service failed!");
	}

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "pose_reset_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	double timeout;

	nh_priv.param("timeout", timeout, 10.0);

	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pose", 1, poseCallback);
	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);
	client = nh.serviceClient<lrm_odometry::SetPose>("set_pose");
	timer = nh_priv.createTimer(ros::Duration(timeout), timerCallback);

	ros::spin();
}
