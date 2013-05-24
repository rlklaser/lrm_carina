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
 * @file plane_segment.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 16, 2013
 *
 */

#pragma clang diagnostic ignored "-Woverloaded-virtual"
#pragma clang diagnostic ignored "-Winvalid-offsetof"
#pragma clang diagnostic ignored "-Wextra-tokens"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//#include <pcl_ros/transforms.h>
//#include <pcl_ros/point_cloud.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/radius_outlier_removal.h>

ros::Publisher pc_pub_gd;
ros::Publisher pc_pub_ngd;
ros::Publisher pc_pub_filter;

ros::Publisher cf_pub;

sensor_msgs::PointCloud2 ground_points;
sensor_msgs::PointCloud2 no_ground_points;
sensor_msgs::PointCloud2 filtered_points;

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	//pcl::PointCloud<pcl::PointXYZ> cloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	pcl::fromROSMsg(*msg, *cloud_filtered);

	//pcl::ModelCoefficients coefficients;
	//pcl::PointIndices inliers;
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	//seg.setMethodType(pcl::SAC_PROSAC);
	seg.setDistanceThreshold(0.25);
	Eigen::Vector3f axis(1,1,0);
	seg.setAxis(axis);
	//seg.setEpsAngle(M_PI/32);

	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);

	if(fabs(coefficients->values[2])>0.05) return;

	// Publish the model coefficients
	cf_pub.publish(*coefficients);

	pcl::ExtractIndices<pcl::PointXYZ> extract_gd;
	extract_gd.setInputCloud(cloud_filtered);
	extract_gd.setIndices(inliers);
	extract_gd.setNegative(false);

	pcl::ExtractIndices<pcl::PointXYZ> extract_ngd;
	extract_ngd.setInputCloud(cloud_filtered);
	extract_ngd.setIndices(inliers);
	extract_ngd.setNegative(true);

	pcl::PointCloud<pcl::PointXYZ> cloud_out;

	extract_gd.filter(cloud_out);
	pcl::toROSMsg(cloud_out, ground_points);
	pc_pub_gd.publish(ground_points);

	extract_ngd.filter(cloud_out);
	pcl::toROSMsg(cloud_out, no_ground_points);
	pc_pub_ngd.publish(no_ground_points);

	pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter(true); // Initializing with true will allow us to extract the removed indices
	rorfilter.setInputCloud(cloud_filtered);
	rorfilter.setRadiusSearch(0.1);
	rorfilter.setMinNeighborsInRadius(5);
	rorfilter.setNegative(false);
	rorfilter.filter(cloud_out);
	// The resulting cloud_out contains all points of cloud_in that have 4 or less neighbors within the 0.1 search radius
	//indices_rem = rorfilter.getRemovedIndices();
	// The indices_rem array indexes all points of cloud_in that have 5 or more neighbors within the 0.1 search radius
	pcl::toROSMsg(cloud_out, filtered_points);
	pc_pub_filter.publish(filtered_points);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "plane_segment");
	ros::NodeHandle n;

	ros::Subscriber pc_sub = n.subscribe("/stereo/points2", 5, pointcloudCallback);

	pc_pub_gd = n.advertise<sensor_msgs::PointCloud2>("ground_points", 5);
	pc_pub_ngd = n.advertise<sensor_msgs::PointCloud2>("no_ground_points", 5);
	pc_pub_filter = n.advertise<sensor_msgs::PointCloud2>("filtered_points", 5);
	cf_pub = n.advertise<pcl::ModelCoefficients>("ground", 1);

	ros::spin();
}

