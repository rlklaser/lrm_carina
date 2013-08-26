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
 * @file region_growing.h
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Aug 26, 2013
 *
 */

#ifndef REGION_GROWING_H_
#define REGION_GROWING_H_

#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/filters/filter.h>
#include <nodelet/nodelet.h>

namespace lrm_stereo {

class RegionGrowingFilter: public pcl_ros::Filter {
protected:
	void filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices, PointCloud2 &output);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}

#include <pluginlib/class_list_macros.h>
typedef lrm_stereo::RegionGrowingFilter RegionGrowingFilter;
PLUGINLIB_EXPORT_CLASS(RegionGrowingFilter, nodelet::Nodelet);

#endif /* REGION_GROWING_H_ */
