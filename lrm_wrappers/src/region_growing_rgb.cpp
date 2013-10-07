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
 * @file region_growing_rgb.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jul 3, 2013
 *
 */

#include <vector>
#include <boost/make_shared.hpp>

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

#include <boost/foreach.hpp>

ros::Publisher pc_pub;

double _k_search;
int _min_size;
int _max_size;
int _neighbours;
double _smoothness;
double _curvature;
tf::TransformListener* tf_listener;

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

	if (pc_pub.getNumSubscribers() == 0)
		return;

	tf::StampedTransform sensorToWorldTf;
	try {
		tf_listener->waitForTransform("/map", msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
		tf_listener->lookupTransform("/map", msg->header.frame_id, msg->header.stamp, sensorToWorldTf);
	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	Eigen::Matrix4f sensorToWorld;
	pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*msg, *cloud_in);

	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud_in);
	normal_estimator.setRadiusSearch(0.2);
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

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cloud_in ;//reg.getColoredCloud();

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

				pcl::transformPointCloud(*cloud_p, *cloud_p, sensorToWorld);

				sensor_msgs::PointCloud2 cloud_out;
				pcl::toROSMsg(*cloud_p, cloud_out);
				cloud_out.header.frame_id = "/map";
				cloud_out.header.stamp = ros::Time::now();
				pc_pub.publish(cloud_out);

			}
		}
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "region_growing_rgb");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	tf_listener = new tf::TransformListener(nh, ros::Duration(30), true);

	ros::Subscriber pc_sub = nh.subscribe("points_in", 10, pointcloudCallback);
	pc_pub = nh.advertise<sensor_msgs::PointCloud2>(nh_priv.getNamespace() + "/points_out", 30);

	nh_priv.param<double>("k_search", _k_search, 50.0);
	nh_priv.param<int>("min_size", _min_size, 100);
	nh_priv.param<int>("max_size", _max_size, 10000);
	nh_priv.param<int>("neighbours", _neighbours, 32.0);
	nh_priv.param<double>("curvature", _curvature, 50.0);
	nh_priv.param<double>("smoothness", _smoothness, 50.0);

	ros::spin();

	delete tf_listener;

	return 0;
}
