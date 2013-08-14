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
 * @file cloud_sum.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jul 1, 2013
 *
 */

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/don.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

double scale1 = 0.1;
double scale2 = 1.0;
double threshold = 0.5;
double segradius = 5.0;

using namespace std;

ros::Publisher pc_pub;
sensor_msgs::PointCloud2 cloud_out;
pcl::PointCloud<pcl::PointXYZRGB> cloud_tot;

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
	//pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::fromROSMsg(*msg, *cloud);

	// Create a search tree, use KDTreee for non-organized data.
	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree;
	if (cloud->isOrganized()) {
		tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>());
	} else {
		tree.reset(new pcl::search::KdTree<pcl::PointXYZRGB>(false));
	}

	// Set the input pointcloud for the search tree
	tree->setInputCloud(cloud);

	// Compute normals using both small and large scales at each point
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);

	/**
	 * NOTE: setting viewpoint is very important, so that we can ensure
	 * normals are all pointed in the same direction!
	 */
	ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

	// calculate normals with the small scale
	cout << "Calculating normals for scale..." << scale1 << endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointNormal>);

	ne.setRadiusSearch(scale1);
	ne.compute(*normals_small_scale);

	// calculate normals with the large scale
	cout << "Calculating normals for scale..." << scale2 << endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>);

	ne.setRadiusSearch(scale2);
	ne.compute(*normals_large_scale);

	// Create output cloud for DoN results
	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointNormal>(*cloud, *doncloud);

	cout << "Calculating DoN... " << endl;
	// Create DoN operator
	pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGB, pcl::PointNormal, pcl::PointNormal> don;
	don.setInputCloud(cloud);
	don.setNormalScaleLarge(normals_large_scale);
	don.setNormalScaleSmall(normals_small_scale);

	if (!don.initCompute()) {
		std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
		return;
	}

	// Compute DoN
	don.computeFeature(*doncloud);

	// Filter by magnitude
	cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

	// Build the condition for filtering
	pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionOr<pcl::PointNormal>());
	range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, threshold)));
	// Build the filter
	pcl::ConditionalRemoval<pcl::PointNormal> condrem(range_cond);
	condrem.setInputCloud(doncloud);

	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

	// Apply filter
	condrem.filter(*doncloud_filtered);

	doncloud = doncloud_filtered;

	// Save filtered output
	std::cout << "Filtered Pointcloud: " << doncloud->points.size() << " data points." << std::endl;

	// Filter by magnitude
	cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

	pcl::search::KdTree<pcl::PointNormal>::Ptr segtree(new pcl::search::KdTree<pcl::PointNormal>);
	segtree->setInputCloud(doncloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;

	ec.setClusterTolerance(segradius);
	ec.setMinClusterSize(50);
	ec.setMaxClusterSize(100000);
	ec.setSearchMethod(segtree);
	ec.setInputCloud(doncloud);
	ec.extract(cluster_indices);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, j++) {
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_cluster_don(new pcl::PointCloud<pcl::PointNormal>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
			cloud_cluster_don->points.push_back(doncloud->points[*pit]);
		}

		cloud_cluster_don->width = int(cloud_cluster_don->points.size());
		cloud_cluster_don->height = 1;
		cloud_cluster_don->is_dense = true;

		sensor_msgs::PointCloud2 cloud_out;
		pcl::toROSMsg(*cloud_cluster_don, cloud_out);

		cloud_out.header = msg->header;
		pc_pub.publish(cloud_out);
		//Save cluster
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size() << " data points." << std::endl;
		//stringstream ss;
		//ss << "don_cluster_" << j << ".pcd";
		//writer.write < pcl::PointNormal > (ss.str(), *cloud_cluster_don, false);
	}

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "cloud_don_segmentation");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::Subscriber pc_sub = nh.subscribe("points_in", 1, pointcloudCallback);
	pc_pub = nh.advertise<sensor_msgs::PointCloud2>(nh_priv.getNamespace() + "/points_out", 1);

	//nh_priv.param<int>("frame_count", _nro_of_frames, 5);
	//ROS_INFO_STREAM("cloud sum withing " << _nro_of_frames << " frames");

	ros::spin();

	return (0);
}
