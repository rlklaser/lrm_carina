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
 * @file relative_pose_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 3, 2013
 *
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

std::string our_frame_id;
geometry_msgs::Pose our_pose;
ros::Publisher pose_pub;

void callback_their(const geometry_msgs::PoseStampedPtr msg) {

	ros::Time now = ros::Time::now();
	//ros::Time now = pose->header.stamp;

	std::string frame_id = msg->header.frame_id;
	geometry_msgs::Pose pose;
	tf::TransformListener listener;

	tf::StampedTransform trans_their_odom;
	try {

		listener.waitForTransform(
				our_frame_id,
				frame_id,
				ros::Time(0),
				ros::Duration(5)
		);

		listener.lookupTransform(
			our_frame_id,
			frame_id,
			ros::Time(0),
			trans_their_odom);

	} catch (tf::TransformException &ex) {
		ROS_ERROR("relative_pose_node: %s", ex.what());
		return;
	}

	ROS_DEBUG_STREAM(
		"our frame:" << our_frame_id << " " <<
		"their frame:" << frame_id << " " <<
		"our:" << our_pose.position.x << " " <<
		"their:" << trans_their_odom.getOrigin().x()
	);

	geometry_msgs::PoseStamped msg_pose;
	msg_pose.header.frame_id = frame_id;
	msg_pose.header.stamp = now;

	tf::Quaternion rotation = trans_their_odom.getRotation();
	tf::Vector3 translation = trans_their_odom.getOrigin();

	msg_pose.pose.position.x = translation.x();
	msg_pose.pose.position.y = translation.y();
	msg_pose.pose.position.z = translation.z();
	msg_pose.pose.orientation.w = rotation.w();
	msg_pose.pose.orientation.x = rotation.x();
	msg_pose.pose.orientation.y = rotation.y();
	msg_pose.pose.orientation.z = rotation.z();

	pose_pub.publish(msg_pose);
}

void callback_our(const geometry_msgs::PoseStampedPtr pose) {
	our_frame_id = "/map";//pose->header.frame_id;
	our_pose = pose->pose;
}

int main(int argc, char ** argv) {

	ros::init(argc, argv, "relative_pose_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::Subscriber subPoseA = nh.subscribe("their/pose", 1, callback_their);
	ros::Subscriber subPoseB = nh.subscribe("our/pose", 1, callback_our);

	pose_pub = nh.advertise<geometry_msgs::PoseStamped>(nh_priv.getNamespace() + "/pose", 1);

	ros::spin();

	return 0;
}
