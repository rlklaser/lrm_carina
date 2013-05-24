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
 * @file cloud_plane_reproject_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Apr 17, 2013
 *
 */

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cFindPlane.h"

ros::Publisher pc_pub;
cFindPlane* findPlane;

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

	PointCloud cloud;
	pcl::fromROSMsg(*msg, cloud);

	float translateY, translateZ, angleScalar = 0.0f;
    findPlane->calculatePlane(cloud, translateY, translateZ, angleScalar);
    findPlane->alignPointCloud(cloud, translateY, translateZ, angleScalar);

    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(cloud, out_msg);

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "cloud_plane_republish_node");
    ros::NodeHandle nh;
    //ros::NodeHandle priv_nh("~");

    findPlane = new cFindPlane(0.1, 1500, Eigen::Vector3f(0.0, 0, 1.0), 1);

    //pubPointCloud = nh.advertise<sensor_msgs::PointCloud2> ("points", 1);
	ros::Subscriber pc_sub = nh.subscribe("points", 5, pointcloudCallback);
	pc_pub = nh.advertise<sensor_msgs::PointCloud2>("points_plane", 5);

    ros::spin();

}
