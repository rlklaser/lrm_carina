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

void lrm_stereo::RegionGrowingFilter::filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices, PointCloud2 &output) {
	//pcl::PointCloud<pcl::PointXYZ> cloud_in;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*input, *cloud_in);

	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud_in);
	//normal_estimator.setKSearch(_k_search);
	normal_estimator.setRadiusSearch(0.2);
	normal_estimator.compute(*normals);

	/*
	 pcl::IndicesPtr indices(new std::vector<int>);
	 pcl::PassThrough<pcl::PointXYZ> pass;
	 pass.setInputCloud(cloud_in);
	 pass.setFilterFieldName("z");
	 pass.setFilterLimits(0.0, 1.0);
	 pass.filter(*indices);
	 */

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
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

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = reg.getColoredCloud();

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
			//BOOST_FOREACH(int i, indices.indices) {
			//	pcl::PointXYZ* pt = &cloud_in->points[i];
			//}
			k++;

			if(indice.indices.size()>0) {

				pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>(indice);

				extract.setIndices(inliers);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
				extract.filter (*cloud_p);

				//pcl::transformPointCloud(*cloud_p, *cloud_p, sensorToWorld);

				//sensor_msgs::PointCloud2 cloud_out;
				pcl::toROSMsg(*cloud_p, output);
				//output.header.frame_id = "/map";
				//cloud_out.header.frame_id = msg->header.frame_id;
				//output.header.stamp = ros::Time::now();
				//pc_pub.publish(cloud_out);

			}
		}

		//out all cloud, red points is the ones not segmented

		/*
		 sensor_msgs::PointCloud2 cloud_out;
		 pcl::toROSMsg(*cloud, cloud_out);
		 //cloud_out.header.frame_id = "/map";
		 //cloud_out.header.stamp = ros::Time::now();
		 cloud_out.header = msg->header;
		 pc_pub.publish(cloud_out);
		 */
	}
}

