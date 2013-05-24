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
 * @file footprint_eraser.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 22, 2013
 *
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <vxl_config.h>
#include <vcl_iostream.h>

#include <vgl/vgl_distance.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_vector_3d.h>


ros::ServiceClient client;
geometry_msgs::PoseWithCovariance last_pose;
double _min_distance_to_update;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

}

void odomCallback(const nav_msgs::Odometry::ConstPtr & msg) {

	octomap_msgs::BoundingBoxQuery srv;
	geometry_msgs::Point min;
	geometry_msgs::Point max;

    vgl_point_3d<double> pt1;
    vgl_point_3d<double> pt2;

    pt1.set(last_pose.pose.position.x, last_pose.pose.position.y, last_pose.pose.position.z);
    pt2.set(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    double dist = vgl_distance(pt1, pt2);

    if(dist>_min_distance_to_update) {
        srv.request.max = max;
        srv.request.min = min;

        if (!client.call(srv)) {
            ROS_ERROR_STREAM("footprint_eraser service failed!");
        }

        last_pose = msg->pose;
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "footprint_eraser");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

    _min_distance_to_update = 0.05;

	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);

	client = nh.serviceClient<octomap_msgs::BoundingBoxQuery>("clear_bbx");

	ros::spin();

	return 0;
}
