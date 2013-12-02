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
 * @file region_growing.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jun 2, 2013
 *
 */

#include "lrm_stereo/region_growing.h"

namespace lrm_stereo {

RegionGrowingFilter::RegionGrowingFilter(ros::NodeHandle& nh, ros::NodeHandle& nh_priv) :
		nh_(nh), nh_priv_(nh_priv) {

	tf_listener_.reset(new tf::TransformListener(nh_));

	nh_priv_.param("queue_size", _queue_size, 100);

	cloud_sub_ = nh_.subscribe("points_in", _queue_size, &RegionGrowingFilter::pointcloudCallback, this);
	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_priv_.getNamespace() + "/points_out", 1);

	nh_priv_.param("radius", _radius, 0.2);
	nh_priv_.param("k_search", _k_search, 30.0); //50
	nh_priv_.param("min_size", _min_size, 100);
	nh_priv_.param("max_size", _max_size, 5000); //100000
	nh_priv_.param("neighbours", _neighbours, 30);
	nh_priv_.param("curvature", _curvature, 2.0); //50
	nh_priv_.param("smoothness", _smoothness, 35.0); //50
	nh_priv_.param("to_map", _to_map, true);
	nh_priv_.param("map_frame_id", _map_frame_id, std::string("/map"));
	nh_priv_.param("use_cloud_color", _use_cloud_color, false);

	dynamic_reconfigure::Server<lrm_stereo::RegionGrowingFilterConfig>::CallbackType
		f = boost::bind(&RegionGrowingFilter::reconfig, this, _1, _2);
	srv_.setCallback(f);
}

void RegionGrowingFilter::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

	if (cloud_pub_.getNumSubscribers() == 0)
		return;

	Eigen::Matrix4f sensorToWorld;

	if (_to_map) {
		tf::StampedTransform sensorToWorldTf;
		try {
			tf_listener_->waitForTransform(_map_frame_id, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
			tf_listener_->lookupTransform(_map_frame_id, msg->header.frame_id, msg->header.stamp, sensorToWorldTf);
		} catch (tf::TransformException& ex) {
			ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
			return;
		}
		pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*msg, *cloud_in);

	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud_in);
	normal_estimator.setRadiusSearch(_radius);
	normal_estimator.compute(*normals);

	pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
	reg.setMinClusterSize(_min_size);
	reg.setMaxClusterSize(_max_size);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(_neighbours);
	reg.setInputCloud(cloud_in);

	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(_smoothness / 180.0 * M_PI);
	reg.setCurvatureThreshold(_curvature);

	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	if (_use_cloud_color) {
		cloud = cloud_in;
	} else {
		cloud = reg.getColoredCloud();
	}

	ROS_DEBUG_STREAM("Number of clusters is equal to " << clusters.size());

	if (cloud) {
		int k = 0;

		ROS_DEBUG_STREAM("cloud in sz " << cloud_in->points.size() << " : cloud out sz " << cloud->points.size());

		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(cloud);
		extract.setNegative(false);

		BOOST_FOREACH(pcl::PointIndices indice, clusters)
		{
			ROS_DEBUG_STREAM("Number of points in cluster " << k << ":" << indice.indices.size());
			k++;

			if(indice.indices.size()>0) {

				pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>(indice);

				extract.setIndices(inliers);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
				extract.filter (*cloud_p);

				if(_to_map) {
					pcl::transformPointCloud(*cloud_p, *cloud_p, sensorToWorld);
				}

				sensor_msgs::PointCloud2 cloud_out;
				pcl::toROSMsg(*cloud_p, cloud_out);
				cloud_out.header.frame_id = _to_map ? _map_frame_id : msg->header.frame_id;
				cloud_out.header.stamp = msg->header.stamp; // ros::Time::now();
				cloud_pub_.publish(cloud_out);

			}
		}
	}
}

void RegionGrowingFilter::reconfig(lrm_stereo::RegionGrowingFilterConfig &config, uint32_t level) {
	_use_cloud_color = config.use_cloud_color;
}

}
