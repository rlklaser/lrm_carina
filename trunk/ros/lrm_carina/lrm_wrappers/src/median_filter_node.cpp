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
 * @file median_filter_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jul 8, 2013
 *
 */

#include <ros/ros.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/common/common.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "pcl/segmentation/extract_clusters.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"

#include <pcl/filters/median_filter.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::search::KdTree<PointT> KdTree;

class MedianFilterNode {
protected:
	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;

public:

	ros::Subscriber sub_;
	ros::Publisher pub_;

	////////////////////////////////////////////////////////////////////////////////
	MedianFilterNode(ros::NodeHandle &n) :
			nh_(n), priv_nh_("~") {

		sub_ = nh_.subscribe("points_in", 1, &MedianFilterNode::cloud_cb, this);
		pub_ = nh_.advertise<sensor_msgs::PointCloud2>(priv_nh_.getNamespace() + "/points_out", 5);
	}

	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& pc) {
		pcl::PointCloud<PointT> cloud_out;
		pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
		pcl::fromROSMsg(*pc, *cloud_in);

		pcl::MedianFilter<PointT> mf;
		mf.setInputCloud(cloud_in);
		mf.setWindowSize(32);
		mf.setMaxAllowedMovement(0.1);
		mf.applyFilter(cloud_out);

		sensor_msgs::PointCloud2 out;
		toROSMsg(cloud_out, out);
		pub_.publish(out);
	}
};

/*

pcl::PointCloud<PointT>::ConstPtr&
mean_filter(std::vector<pcl::PointCloud<PointT>::ConstPtr> &cloud) {

	// We assume all the point clouds have the same number of points, and that they have the same order
	// If that isn't the case, this will produce random results
	unsigned int n_points = cloud[0]->size();
	unsigned int n_measurements = cloud.size();

	pcl::PointCloud<Point> cloud_filtered;
	cloud_filtered.header = cloud[0]->header;
	// zzz Fix later: cloud_filtered.points.size = cloud[0].points.size();
	cloud_filtered.is_dense = cloud[0]->is_dense;

	// zzz should be fixed! do not copie the entire point cloud !!!
	cloud_filtered = *cloud[0];

	std::vector < Point > points(n_measurements);
	for (unsigned int pp = 0; pp < n_points; pp++) {
		// Filter pixel by pixel
		for (unsigned int ii = 0; ii < n_measurements; ii++) {
			points[ii] = cloud[ii]->points[pp];
		}
		// Now filter

		// Simple mean filter
		Point point_mean = Point(0, 0, 0);
		for (unsigned int ii = 0; ii < n_measurements; ii++) {
			point_mean.x += points[ii].x / n_measurements;
			point_mean.y += points[ii].y / n_measurements;
			point_mean.z += points[ii].z / n_measurements;
		}

		// Done filtering for this pixel
		cloud_filtered.points[pp] = point_mean;

	}

	mean_cloud_ptr_ = boost::make_shared < pcl::PointCloud<Point> > (cloud_filtered);
	return mean_cloud_ptr_;

}

pcl::PointCloud<PointT>::ConstPtr&
median_filter(std::vector<pcl::PointCloud<PointT>::ConstPtr> &cloud) {

	// We assume all the point clouds have the same number of points, and that they have the same order
	// If that isn't the case, this will produce random results
	unsigned int n_points = cloud[0]->size();
	unsigned int n_measurements = cloud.size();
	unsigned int median_index = floor(n_measurements / 2);

	std::vector<double> points_x(n_measurements);
	std::vector<double> points_y(n_measurements);
	std::vector<double> points_z(n_measurements);

	pcl::PointCloud<Point> cloud_filtered;
	cloud_filtered.header = cloud[0]->header;
	// zzz Fix later: cloud_filtered.points.size = cloud[0].points.size();
	cloud_filtered.is_dense = cloud[0]->is_dense;

	cloud_filtered = *cloud[0];

	std::vector < Point > points(n_measurements);
	for (unsigned int pp = 0; pp < n_points; pp++) {
		// Filter pixel by pixel
		for (unsigned int ii = 0; ii < n_measurements; ii++) {
			points[ii] = cloud[ii]->points[pp];
		}
		// Now filter

		// median filter
		Point point_median = Point(0, 0, 0);
		for (unsigned int ii = 0; ii < n_measurements; ii++) {
			points_x[ii] = points[ii].x;
			points_y[ii] = points[ii].y;
			points_z[ii] = points[ii].z;
		}

		sort(points_x.begin(), points_x.end());
		sort(points_y.begin(), points_y.end());
		sort(points_z.begin(), points_z.end());

		point_median.x = points_x[median_index];
		point_median.y = points_y[median_index];
		point_median.z = points_z[median_index];

		// Done filtering for this pixel
		cloud_filtered.points[pp] = point_median;

	}

	median_cloud_ptr_ = boost::make_shared < pcl::PointCloud<Point> > (cloud_filtered);
	return median_cloud_ptr_;

}

double avg_points_z(pcl::PointCloud<PointT>::ConstPtr &cloud) {
	// average over the 'z' coordinate of the table top plane
	// get the 'x' nearest front
	unsigned int n_points = (unsigned int) cloud->points.size();
	double z_coord = 0.0;
	double x_coord = cloud->points[0].x;
	for (size_t i = 0; i < n_points; ++i) {
		z_coord += cloud->points[i].z;

		if (x_coord > cloud->points[i].x)
			x_coord = cloud->points[i].x;
	}
	z_coord = z_coord / n_points;

	return z_coord;
}

*/

/*
int getClosestPositionAndErase(std::vector<Eigen::Vector3f>& sampling_positions, geometry_msgs::PoseWithCovarianceStamped robot_pose) {
	float dist = 100.0;
	int index = 0;

	for (unsigned int ii = 0; ii < sampling_positions.size(); ii++) {
		float tmp_dist = sqrt(pow(robot_pose.pose.pose.position.x - sampling_positions[ii].x(), 2) + pow(robot_pose.pose.pose.position.y - sampling_positions[ii].y(), 2) + pow(robot_pose.pose.pose.position.z - sampling_positions[ii].z(), 2));
		if (tmp_dist < dist) {
			dist = tmp_dist;
			index = ii;
		}
	}

	// erase the sample point from the list, which is closest to
	// the scan position of possible sample positions.
	sampling_positions.erase(sampling_positions.begin() + index);

	return index;
}
*/

int main(int argc, char** argv) {
	ros::init(argc, argv, "median_filter_node");
	ros::NodeHandle n;
	MedianFilterNode mm(n);
	ros::spin();
	return (0);
}
