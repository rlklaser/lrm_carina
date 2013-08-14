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
 * @file cloud_cut.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jan 3, 2013
 *
 */

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

ros::Publisher pc_pub;
ros::Publisher cf_pub;

//using namespace pcl_ros;
sensor_msgs::PointCloud2 filtered;

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	/*
	 PointCloud pc;
	 pcl::fromROSMsg(*msg, pc);

	 pcl::PassThrough < pcl::PointXYZ > pass;
	 pass.setFilterFieldName("z");
	 pass.setFilterLimits(0.5, 1.0);

	 pass.setInputCloud(pc.makeShared());
	 pass.filter(pc);

	 pcl::toROSMsg(pc, out_msg);

	 */
	if (pc_pub.getNumSubscribers() > 0 || cf_pub.getNumSubscribers() > 0)
	{
		pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
		sor.setInputCloud(msg);
		sor.setLeafSize(0.05, 0.05, 0.05);
		sor.filter(filtered);

		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::fromROSMsg(filtered, cloud);
		// Create the filtering object
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor2;
		sor2.setInputCloud(cloud.makeShared());
		sor2.setMeanK(50);
		sor2.setStddevMulThresh(1.0);
		sor2.filter(cloud);
		pcl::toROSMsg(cloud, filtered);

		filtered.header = msg->header;
		pc_pub.publish(filtered);
	}
	/*
	 pcl::PointCloud<pcl::PointXYZ> cloud;
	 pcl::fromROSMsg(filtered, cloud);

	 pcl::ModelCoefficients coefficients;
	 pcl::PointIndices inliers;
	 // Create the segmentation object
	 pcl::SACSegmentation<pcl::PointXYZ> seg;
	 // Optional
	 seg.setOptimizeCoefficients(true);
	 // Mandatory
	 seg.setModelType(pcl::SACMODEL_PLANE);
	 seg.setMethodType(pcl::SAC_RANSAC);
	 //seg.setMethodType(pcl::SAC_PROSAC);
	 seg.setDistanceThreshold(0.05);

	 seg.setInputCloud(cloud.makeShared());
	 seg.segment(inliers, coefficients);

	 // Publish the model coefficients
	 cf_pub.publish(coefficients);
	 */
}

void timerCallback(const ros::TimerEvent& t)
{
	if (filtered.header.stamp != ros::Time(0) && cf_pub.getNumSubscribers() > 0)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::fromROSMsg(filtered, cloud);

		pcl::ModelCoefficients coefficients;
		pcl::PointIndices inliers;
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		//seg.setMethodType(pcl::SAC_PROSAC);
		seg.setDistanceThreshold(0.05);

		seg.setInputCloud(cloud.makeShared());
		seg.segment(inliers, coefficients);

		// Publish the model coefficients
		cf_pub.publish(coefficients);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_cut");
	ros::NodeHandle n;

	ros::Subscriber pc_sub = n.subscribe("points", 5, pointcloudCallback);

	pc_pub = n.advertise<sensor_msgs::PointCloud2>("points_cut", 5);
	cf_pub = n.advertise<pcl::ModelCoefficients>("ground", 1);

	ros::Timer plane_timer = n.createTimer(ros::Duration(1.0), timerCallback);
	plane_timer.start();

	ros::spin();
}
