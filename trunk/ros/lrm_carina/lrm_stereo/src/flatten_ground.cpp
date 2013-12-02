/*
 *  Copyright (C) 2012-2013, Laboratorio de Robotica Movel - ICMC/USP
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
 * @file flatten_ground.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jun 2, 2013
 *
 */

#include "lrm_stereo/flatten_ground.h"

namespace lrm_stereo {

FlattenGroundFilter::FlattenGroundFilter(ros::NodeHandle& nh, ros::NodeHandle& nh_priv) :
		nh_(nh), nh_priv_(nh_priv) {

	nh_priv_.param("queue_size", _queue_size, 10);

	cloud_sub_ = nh_.subscribe("points_in", _queue_size, &FlattenGroundFilter::pointcloudCallback, this);
	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_priv_.getNamespace() + "/points_out", 1);
}

void FlattenGroundFilter::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

	if (cloud_pub_.getNumSubscribers() == 0)
		return;

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_out;

	sensor_msgs::PointCloud2 msg_out;

	pcl::fromROSMsg(*msg, cloud);

	itrtor end = cloud.points.end();
	for (itrtor itr = cloud.points.begin(); itr != end; ++itr) {
		//if(itr->z<0.15) {
			itr->z = 0;
			cloud_out.points.push_back(*itr);
		//}
	}

	pcl::toROSMsg(cloud_out, msg_out);
	msg_out.header = msg->header;
	cloud_pub_.publish(msg_out);
}

}
