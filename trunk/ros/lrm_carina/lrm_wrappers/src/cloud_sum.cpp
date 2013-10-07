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

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

int _nro_of_frames;
bool _incremental;

ros::Publisher pc_pub;

sensor_msgs::PointCloud2 cloud_out;
pcl::PointCloud<pcl::PointXYZRGB> cloud_tot;
int frame_count;
tf::TransformListener* tf_listener;

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

	if (pc_pub.getNumSubscribers() == 0)
		return;

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::fromROSMsg(*msg, cloud);

	tf::StampedTransform sensorToWorldTf;
	try {
		tf_listener->waitForTransform("/map", msg->header.frame_id, msg->header.stamp, ros::Duration(0.5));
		tf_listener->lookupTransform("/map", msg->header.frame_id, msg->header.stamp, sensorToWorldTf);
	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
		/*
		 ROS_WARN_STREAM("Transform error of sensor data: " << ex.what() << ", retrying with latest");
		 try {
		 tf_listener->lookupTransform("/map", msg->header.frame_id, ros::Time(0), sensorToWorldTf);
		 } catch (tf::TransformException& ex) {
		 ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
		 return;
		 }
		 */
	}

	Eigen::Matrix4f sensorToWorld;
	pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

	//sum to a fixed frame
	pcl::transformPointCloud(cloud, cloud, sensorToWorld);
	cloud_tot += cloud;

	frame_count++;

	if (frame_count == _nro_of_frames || _incremental) {

		//back to sensor frame
//		Eigen::Matrix4f worldToSensor;
//		worldToSensor = -sensorToWorld.inverse();
//		pcl::transformPointCloud(cloud_tot, cloud, worldToSensor);
//		pcl::toROSMsg(cloud, cloud_out);

		pcl::toROSMsg(cloud_tot, cloud_out);
		cloud_out.header.frame_id = "/map";
//		cloud_out.header.frame_id = msg->header.frame_id;
		cloud_out.header.stamp = ros::Time::now();
		pc_pub.publish(cloud_out);

		if (frame_count == _nro_of_frames) {
			frame_count = 0;
			cloud_tot.clear();
		}
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "cloud_sum");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	cloud_tot.clear();
	frame_count = 0;

	tf_listener = new tf::TransformListener(nh, ros::Duration(30), true);

	ros::Subscriber pc_sub = nh.subscribe("points_in", 5, pointcloudCallback);
	pc_pub = nh.advertise<sensor_msgs::PointCloud2>(nh_priv.getNamespace() + "/points_out", 10);
	nh_priv.param<int>("frame_count", _nro_of_frames, 5);
	nh_priv.param<bool>("incremental", _incremental, false);

	ROS_INFO_STREAM("cloud sum within " << _nro_of_frames << " frames");

	ros::spin();

	delete tf_listener;

	return (0);
}
