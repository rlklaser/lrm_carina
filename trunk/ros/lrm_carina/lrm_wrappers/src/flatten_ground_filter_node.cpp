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
 * @file flatten_ground_filter_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Oct 4, 2013
 *
 */


#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <boost/assign.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>

#include <math.h>

#include <Eigen/Core>

typedef pcl::PointCloud<pcl::PointXYZRGB>::iterator itrtor;

ros::Publisher pc_pub;
sensor_msgs::PointCloud2 cloud_out;

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

	if (pc_pub.getNumSubscribers() == 0 )
		return;

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_out;

	sensor_msgs::PointCloud2 msg_out;

	pcl::fromROSMsg(*msg, cloud);


	itrtor end = cloud.points.end();
	for (itrtor itr = cloud.points.begin(); itr != end; ++itr )
	{
		itr->z = 0;
		cloud_out.points.push_back(*itr);
	}

	pcl::toROSMsg(cloud_out, msg_out);
	msg_out.header = msg->header;
	pc_pub.publish(msg_out);

}



int main(int argc, char** argv) {
	ros::init(argc, argv, "flatten_ground_filter_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::Subscriber pc_sub = nh.subscribe("points_in", 1, pointcloudCallback);

	pc_pub = nh.advertise<sensor_msgs::PointCloud2>(nh_priv.getNamespace() + "/points_out", 10);

	ros::spin();

	return 0;
}
