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
 * @file zigzag_cut.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jun 2, 2013
 *
 */

#include "lrm_stereo/zigzag_cut.h"

namespace lrm_stereo {

ZigzagCutFilter::ZigzagCutFilter(ros::NodeHandle& nh, ros::NodeHandle& nh_priv) :
		nh_(nh), nh_priv_(nh_priv) {

	nh_priv_.param("queue_size", _queue_size, 100);

	cloud_sub_ = nh_.subscribe("points_in", _queue_size, &ZigzagCutFilter::pointcloudCallback, this);
	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_priv_.getNamespace() + "/points_out", 1);
	cloud_rem_pub_ = nh.advertise<sensor_msgs::PointCloud2>(nh_priv.getNamespace() + "/points_out_removed", 1);
}

void ZigzagCutFilter::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

	if (cloud_pub_.getNumSubscribers() == 0)
		return;

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_rem_out;

	sensor_msgs::PointCloud2 msg_out;

	pcl::fromROSMsg(*msg, cloud);

	double min_x = 99999999;
	double max_x = 0;
	long qtd = 0;
	double x;
	double tot = 0;
	double mean = 0;
	double var = 0;
	double stdev = 0;
	double centered_x;

	itrtor end = cloud.points.end();
	for (itrtor itr = cloud.points.begin(); itr != end; ++itr) {
		qtd++;
		x = itr->x;
		tot += x;

		if (x > max_x)
			max_x = x;
		if (x < min_x)
			min_x = x;
	}

	tot = tot - (min_x * qtd);
	mean = tot / qtd;

	tot = 0;
	for (itrtor itr = cloud.points.begin(); itr != end; ++itr) {
		x = itr->x;
		centered_x = x - min_x;
		tot += (centered_x - mean) * (centered_x - mean);
	}

	var = tot / qtd;
	stdev = sqrt(var);

	qtd = 0;
	for (itrtor itr = cloud.points.begin(); itr != end; ++itr) {
		x = itr->x;
		centered_x = x - min_x;
		if (centered_x > (mean - stdev) && centered_x < (mean + stdev)) {
			cloud_out.points.push_back(*itr);
		} else {
			cloud_rem_out.points.push_back(*itr);
			qtd++;
		}
	}

	pcl::toROSMsg(cloud_out, msg_out);
	msg_out.header = msg->header;
	cloud_pub_.publish(msg_out);

	pcl::toROSMsg(cloud_rem_out, msg_out);
	msg_out.header = msg->header;
	cloud_rem_pub_.publish(msg_out);

	//std::cout << "removed " << qtd << std::endl;
}

}
