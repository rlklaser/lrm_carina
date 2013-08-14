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
 * @file bilateral_filter_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 28, 2013
 *
 */

// ROS core
#include <ros/ros.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/common/common.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "pcl/segmentation/extract_clusters.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"

#include <pcl/filters/bilateral.h>

using namespace std;

typedef pcl::PointXYZI PointT;
typedef pcl::search::KdTree<PointT> KdTree;

class BilateralFilterNode {
protected:
	ros::NodeHandle nh_;
	ros::NodeHandle priv_nh_;

	double sigma_r;
	double sigma_s;

public:
	string output_cluster_topic_, input_cloud_topic_, output_cloud_cluster_topic_;

	ros::Subscriber sub_;
	ros::Publisher pub_;

	//sensor_msgs::PointCloud2 cloud_out_;
	//pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_in_;

	////////////////////////////////////////////////////////////////////////////////
	BilateralFilterNode(ros::NodeHandle &n) :
			nh_(n), priv_nh_("~") {

		sub_ = nh_.subscribe("in_cloud", 1, &BilateralFilterNode::cloud_cb, this);
		pub_ = nh_.advertise<sensor_msgs::PointCloud2>("out_cloud", 5);

		sigma_r = 50.0;
		sigma_s = 5.0;
	}

	////////////////////////////////////////////////////////////////////////////////
	// cloud_cb (!)
	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& pc) {
		//PointT in_cloud;
		pcl::PointCloud<PointT> cloud_out;

		pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>);
		pcl::fromROSMsg(*pc, *cloud_in);
		//cloud_in_ = boost::make_shared<const PointT>(in_cloud);

		KdTree::Ptr tree = boost::make_shared<KdTree>();

        PointT pt = cloud_in->points[0];

        std::cout << pt.intensity << std::endl;

		pcl::BilateralFilter<PointT> bf;
		bf.setInputCloud(cloud_in);
		bf.setSearchMethod(tree);
		bf.setHalfSize(sigma_s);
		bf.setStdDev(sigma_r);
		bf.filter(cloud_out);

		sensor_msgs::PointCloud2 out;
		toROSMsg(cloud_out, out);
		pub_.publish(out);
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "bilateral_filter_node");
	ros::NodeHandle n;
	BilateralFilterNode mm(n);
	ros::spin();
	return (0);
}
