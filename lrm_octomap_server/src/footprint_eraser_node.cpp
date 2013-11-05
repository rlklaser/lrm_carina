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
 * @file footprint_eraser.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 22, 2013
 *
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <octomap/math/Vector3.h>
/*
 #include <vxl_config.h>
 #include <vcl_iostream.h>
 #include <vgl/vgl_distance.h>
 #include <vgl/vgl_point_3d.h>
 #include <vgl/vgl_vector_3d.h>
 */

ros::ServiceClient _client;
geometry_msgs::PoseWithCovariance _last_pose;
double _min_distance_to_update;
ros::Time _last_clear;
double _timeout;

tf::StampedTransform _transform_min;
tf::StampedTransform _transform_max;

//void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//}

void odomCallback(const nav_msgs::Odometry::ConstPtr & msg) {

	octomap_msgs::BoundingBoxQuery srv;
	geometry_msgs::Point min;
	geometry_msgs::Point max;

	/*
	 vgl_point_3d<double> pt1;
	 vgl_point_3d<double> pt2;
	 pt1.set(last_pose.pose.position.x, last_pose.pose.position.y, last_pose.pose.position.z);
	 pt2.set(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	 double dist = vgl_distance(pt1, pt2);
	 */
	octomath::Vector3 pt1(_last_pose.pose.position.x, _last_pose.pose.position.y, _last_pose.pose.position.z);
	octomath::Vector3 pt2(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

	double dist = pt1.distance(pt2);

	bool timeouted = (ros::Time::now() - _last_clear) > ros::Duration(_timeout);
	//if (timeouted)
	//	ROS_WARN_STREAM("clearing, timeouted...");

	if (dist > _min_distance_to_update || timeouted) {

		tf::Quaternion qt = tf::createQuaternionFromYaw(msg->pose.pose.orientation.z);
		tf::Transform trans_odom(qt, tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
		tf::Transform trans_min;
		tf::Transform trans_max;
		//min = msg->pose.pose.position * transform_min;
		//max = msg->pose.pose.position * transform_max;

		trans_min = trans_odom * _transform_min;
		trans_max = trans_odom * _transform_max;

		max.x = trans_max.getOrigin().x();
		max.y = trans_max.getOrigin().y();
		max.z = trans_max.getOrigin().z();

		min.x = trans_min.getOrigin().x();
		min.y = trans_min.getOrigin().y();
		min.z = trans_min.getOrigin().z();

		srv.request.max = max;
		srv.request.min = min;

		ROS_DEBUG_STREAM("call service (" << min.x << ":" << min.y << ":" << min.z << ")(" << max.x << ":" << max.y << ":" << max.z << ")");

		if (!_client.call(srv)) {
			ROS_ERROR_STREAM("footprint_eraser service failed!");
		}

		_last_pose = msg->pose;
		_last_clear = ros::Time::now();

	} else {
		ROS_DEBUG_STREAM("nothing to do");
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "footprint_eraser_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	//_min_distance_to_update;

	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 30, odomCallback);

	_client = nh.serviceClient<octomap_msgs::BoundingBoxQuery>("clear_bbx");

	nh_priv.param<double>("min_distance", _min_distance_to_update, 0.5);
	nh_priv.param<double>("timeout", _timeout, 30);

	_last_clear = ros::Time::now();

	tf::TransformListener listener;

	try {
		listener.waitForTransform("base_footprint", "bbx_base_rear_left_link", ros::Time(0), ros::Duration(5.0));
	} catch (tf::TransformException &ex) {
		ROS_ERROR("footprint_eraser wait: %s", ex.what());
	}

	try {
		listener.waitForTransform("base_footprint", "bbx_top_front_right_link", ros::Time(0), ros::Duration(5.0));
	} catch (tf::TransformException &ex) {
		ROS_ERROR("footprint_eraser wait: %s", ex.what());
	}

	try {
		listener.lookupTransform("base_footprint", "bbx_base_rear_left_link", ros::Time(0), _transform_min);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("footprint_eraser: %s", ex.what());
	}

	try {
		listener.lookupTransform("base_footprint", "bbx_top_front_right_link", ros::Time(0), _transform_max);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("footprint_eraser: %s", ex.what());
	}

	ROS_INFO_STREAM("footprint_eraser start spinning...");

	ros::spin();

	return 0;
}
