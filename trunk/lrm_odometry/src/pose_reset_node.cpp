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

ros::ServiceClient client;
ros::Timer timer;
geometry_msgs::Pose2D pose;

void poseCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg) {
	pose.x = msg->pose.pose.position.x;
	pose.y = msg->pose.pose.position.y;
	pose.theta = msg->pose.pose.orientation.z;
}

void timerCallback(const ros::TimerEvent& e) {
	lrm_odometry::SetPose srv;

	srv.request.pose.x = pose.x;
	srv.request.pose.y = pose.x;
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

	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pose2d", 1, poseCallback);
	client = nh.serviceClient<lrm_odometry::SetPose>("set_pose");
	timer = nh_priv.createTimer(ros::Duration(timeout), timerCallback);

	ros::spin();
}
