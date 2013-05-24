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
 * @file point_cloud_transform_nodelet.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 22, 2013
 *
 */

#pragma clang diagnostic ignored "-Winvalid-offsetof"

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

namespace lrm_navigation {

typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

class TransformCloudNodelet: public nodelet::Nodelet {

	ros::Subscriber sub;
	ros::Publisher pub;
	tf::TransformListener listener;
	std::string _base_frame_id;

	virtual void onInit();
	void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};

void TransformCloudNodelet::onInit() {
	ros::NodeHandle &nh = getNodeHandle();
	ros::NodeHandle &priv_nh = getPrivateNodeHandle();

	ros::Subscriber sub = nh.subscribe("points_in", 1, &TransformCloudNodelet::callback, this);
	pub = nh.advertise<sensor_msgs::PointCloud2>(priv_nh.getNamespace() + "/points_out", 1);

	_base_frame_id = "base_footprint";

	try {
		listener.waitForTransform("base_footprint", "stereo_camera", ros::Time(0), ros::Duration(10.0));
	} catch (tf::TransformException &ex) {
		ROS_ERROR("TransformCloudNodelet wait: %s", ex.what());
	}
}

void TransformCloudNodelet::callback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
	/*
	 sensor_msgs::PointCloud msg_in;
	 sensor_msgs::PointCloud msg_out;

	 //msg_in = *msg;

	 tf::StampedTransform trans_base;
	 try {
	 //listener.lookupTransform("base_footprint", "stereo_camera", ros::Time(0), trans_base);
	 listener.transformPointCloud("base_footprint", *msg, msg_out);
	 //listener.transformPointCloud(msg, msg_out, trans_base);
	 //pcl::transformPointCloud(msg, msg_out, trans_base);

	 } catch (tf::TransformException &ex) {
	 ROS_ERROR("TransformCloudNodelet: %s", ex.what());
	 }

	 pub.publish(msg_out);
	 */
	PCLPointCloud pc; // input cloud for filtering and ground-detection
	pcl::fromROSMsg(*cloud, pc);

	tf::StampedTransform sensorToBaseTf;
	try {
		listener.lookupTransform(_base_frame_id, cloud->header.frame_id, cloud->header.stamp, sensorToBaseTf);
	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	Eigen::Matrix4f sensorToBase;
	pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
	pcl::transformPointCloud(pc, pc, sensorToBase);


}

}

// Register nodelet
#include <pluginlib/class_list_macros.h>
//PLUGINLIB_DECLARE_CLASS(lrm_navigation, transform_cloud, lrm_navigation::TransformCloudNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(lrm_navigation::TransformCloudNodelet, nodelet::Nodelet)

