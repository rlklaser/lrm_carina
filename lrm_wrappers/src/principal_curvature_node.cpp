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
 * @file principal_curvature_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Sep 30, 2013
 *
 */

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>


#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <visualization_msgs/Marker.h>

ros::Publisher pc_pub;

sensor_msgs::PointCloud2 cloud_out;
pcl::PointCloud<pcl::PointXYZRGB> cloud_tot;
tf::TransformListener* tf_listener;

ros::Publisher marker_pub;

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

	if (pc_pub.getNumSubscribers() == 0)
		return;

	//pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg, *cloud);

	// Compute the normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setInputCloud(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::Normal>);

	normal_estimation.setRadiusSearch(0.03);

	normal_estimation.compute(*cloud_with_normals);

	// Setup the principal curvatures computation
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

	// Provide the original point cloud (without normals)
	principal_curvatures_estimation.setInputCloud(cloud);

	// Provide the point cloud with normals
	principal_curvatures_estimation.setInputNormals(cloud_with_normals);

	// Use the same KdTree from the normal estimation
	principal_curvatures_estimation.setSearchMethod(tree);
	principal_curvatures_estimation.setRadiusSearch(1.0);

	// Actually compute the principal curvatures
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>());
	principal_curvatures_estimation.compute(*principal_curvatures);

	//std::cout << "input points.size (): " << cloud->points.size()  << " output points.size (): " << principal_curvatures->points.size() << std::endl;

	visualization_msgs::Marker marker;

	marker.header.frame_id = msg->header.frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = "principal_curve_marker";
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.color.a = 0.8;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	for(int i=0; i<principal_curvatures->points.size(); i++) {
		geometry_msgs::Point p;

		p.x = principal_curvatures->points[i].principal_curvature_x;
		p.y = principal_curvatures->points[i].principal_curvature_y;
		p.z = principal_curvatures->points[i].principal_curvature_z;

		if(!isnan(p.z) && !isnan(p.y) && !isnan(p.z)) {
			marker.points.push_back(p);
		}
	}

	if(marker.points.size()>0) {
		marker_pub.publish(marker);
	}

	// Display and retrieve the shape context descriptor vector for the 0th point.
	//pcl::PrincipalCurvatures descriptor = principal_curvatures->points[0];
	//std::cout << descriptor << std::endl;

//	pcl::toROSMsg(cloud_tot, cloud_out);
//	cloud_out.header.frame_id = msg->header.frame_id;
//	cloud_out.header.stamp = ros::Time::now();
//	pc_pub.publish(cloud_out);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "principal_curvature_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	//tf_listener = new tf::TransformListener(nh, ros::Duration(30), true);

	ros::Subscriber pc_sub = nh.subscribe("points_in", 1, pointcloudCallback);
	//ros::Subscriber pc_sub = nh.subscribe("/cloud/points_cluster", 1, pointcloudCallback);


	pc_pub = nh.advertise<sensor_msgs::PointCloud2>(nh_priv.getNamespace() + "/points_out", 1);
	//nh_priv.param<int>("frame_count", _nro_of_frames, 5);
	//nh_priv.param<bool>("incremental", _incremental, false);

	marker_pub = nh.advertise<visualization_msgs::Marker>(nh_priv.getNamespace() +  "/visualization_marker", 1);

	//ROS_INFO_STREAM("cloud sum within " << _nro_of_frames << " frames");

	ros::spin();

	return (0);
}
