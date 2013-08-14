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
 * @file cluster_filter_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jul 4, 2013
 *
 */

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <Eigen/Core>

#include <sstream>

ros::Publisher pc_pub;
tf::TransformListener* tf_listener;

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

	if (pc_pub.getNumSubscribers() == 0)
		return;

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*msg, cloud);

	pcl::PointXYZ proj_min;
	pcl::PointXYZ proj_max;
	pcl::getMinMax3D(cloud, proj_min, proj_max);

	// Placeholder for the 3x3 covariance matrix at each surface patch
	Eigen::Matrix3f covariance_matrix;
	// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
	Eigen::Vector4f xyz_centroid;
	// Estimate the XYZ centroid
	pcl::compute3DCentroid(cloud, xyz_centroid);
	// Compute the 3x3 covariance matrix
	pcl::computeCovarianceMatrix(cloud, xyz_centroid, covariance_matrix);

	std::ostringstream s;

	s << "min (x,y,z)(" << proj_min.x << "," << proj_min.y << "," << proj_min.z << ")" <<
	" - centroid (x,y,z)(" << xyz_centroid[0] << "," << xyz_centroid[1] << "," << xyz_centroid[2] << ")" <<
	" - max (x,y,z)(" << proj_max.x << "," << proj_max.y << "," << proj_max.z << ")";


	bool publish = false;

//	if (xyz_centroid[0] < 5.0) {
//		publish = true;
//	} else {
		if (proj_min.z < 0.2 && proj_max.z > 0.3) {
			publish = true;
		}
//	}

	if (publish) {
		pc_pub.publish(*msg);
		//ROS_INFO_STREAM(s.str());
		ROS_DEBUG_STREAM(s.str());
	} else {
		//ROS_WARN_STREAM(s.str());
		ROS_DEBUG_STREAM(s.str());
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "cluster_filter_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	tf_listener = new tf::TransformListener(nh, ros::Duration(30), true);

	ros::Subscriber pc_sub = nh.subscribe("points_in", 20, pointcloudCallback);
	pc_pub = nh.advertise<sensor_msgs::PointCloud2>(nh_priv.getNamespace() + "/points_out", 20);

	ros::spin();

	delete tf_listener;

	return 0;
}
