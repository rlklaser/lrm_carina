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
 * @file segmentation_nodelet.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Oct 2, 2013
 *
 */

#include "lrm_stereo/segmentation_nodelet.h"

lrm_stereo::SegmentationNodelet::SegmentationNodelet()
{
}

lrm_stereo::SegmentationNodelet::SegmentationNodelet(ros::NodeHandle nh) :
		nh_(nh)
{
}

lrm_stereo::SegmentationNodelet::~SegmentationNodelet()
{
}

void lrm_stereo::SegmentationNodelet::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

}

void lrm_stereo::SegmentationNodelet::onInit()
{
	ros::NodeHandle nh_priv("~");

	//boost::bind(&SegmentationNodelet::callback, this, _1);

	sub_ = nh_.subscribe("cloud_in", 1, &lrm_stereo::SegmentationNodelet::callback, this);
	pub_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_priv.getNamespace() + "/points_out", 1);
}

PLUGINLIB_EXPORT_CLASS(lrm_stereo::SegmentationNodelet, nodelet::Nodelet);

