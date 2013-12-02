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
 * @file flatten_ground.h
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Aug 26, 2013
 *
 */

#ifndef FLATTEN_GROUND_H_
#define FLATTEN_GROUND_H_

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

namespace lrm_stereo {

class FlattenGroundFilter {
private:
	ros::NodeHandle& nh_;
	ros::NodeHandle& nh_priv_;
	ros::Publisher cloud_pub_;
	ros::Subscriber cloud_sub_;

	int _queue_size;
protected:
	void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
public:
	FlattenGroundFilter(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
};
}

#endif /* FLATTEN_GROUND_H_ */
