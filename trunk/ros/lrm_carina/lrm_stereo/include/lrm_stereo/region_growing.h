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

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <vector>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <dynamic_reconfigure/server.h>

#include "lrm_stereo/RegionGrowingFilterConfig.h"

namespace lrm_stereo {

class RegionGrowingFilter {
private:
	ros::NodeHandle& nh_;
	ros::NodeHandle& nh_priv_;
	ros::Publisher cloud_pub_;
	ros::Subscriber cloud_sub_;

	dynamic_reconfigure::Server<RegionGrowingFilterConfig> srv_;

	int _queue_size;
	double _radius;
	double _k_search;
	int _min_size;
	int _max_size;
	int _neighbours;
	double _smoothness;
	double _curvature;
	bool _to_map;
	bool _use_cloud_color;
	std::string _map_frame_id;

	boost::shared_ptr<tf::TransformListener> tf_listener_;

protected:
	void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
public:
	RegionGrowingFilter(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
	//virtual ~RegionGrowingFilter();
	void reconfig(RegionGrowingFilterConfig &config, uint32_t level);
};
}

#endif /* REGION_GROWING_H_ */
