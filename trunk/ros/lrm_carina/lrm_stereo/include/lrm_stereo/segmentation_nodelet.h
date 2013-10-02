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
 * @file segmentation_nodelet.h
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Oct 2, 2013
 *
 */

#ifndef SEGMENTATION_NODELET_H_
#define SEGMENTATION_NODELET_H_


#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>

namespace lrm_stereo {

class SegmentationNodelet: public nodelet::Nodelet
{
public:
	SegmentationNodelet();
	SegmentationNodelet(ros::NodeHandle);
	virtual ~SegmentationNodelet();

	virtual void onInit();

private:
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::Publisher pub_;

	void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};

}
#endif /* SEGMENTATION_NODELET_H_ */
