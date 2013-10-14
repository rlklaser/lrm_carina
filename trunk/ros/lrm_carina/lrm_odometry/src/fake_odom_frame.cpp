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
 * @file fake_odom_frame.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Oct 14, 2013
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//#include <boost/thread/mutex.hpp>

boost::mutex _mutex;

double _yaw;
bool _stop_publish;

void odomCallback(const nav_msgs::Odometry::ConstPtr & msg) {
	//boost::unique_lock < boost::mutex > scoped_lock(_mutex);

	_stop_publish = true;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr msg) {
	//boost::unique_lock < boost::mutex > scoped_lock(_mutex);

	_yaw = tf::getYaw(msg->orientation);
}

void publishTF() {
	tf::TransformBroadcaster odom_broadcaster;
/*
	tf::StampedTransform trans_base_enc;
	try {
		listener.lookupTransform("base_footprint", "odom", ros::Time(0), trans_base_enc);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("position_node: %s", ex.what());
		return false;
	}
*/
	tf::Quaternion qt = tf::createQuaternionFromYaw(_yaw);
	tf::Transform trans_odom_encoder(qt, tf::Vector3(0.0, 0.0, 0.0));

	tf::StampedTransform trans_odom_base_st(trans_odom_encoder, ros::Time::now(),
			"/odom", "/base_footprint");

	odom_broadcaster.sendTransform(trans_odom_base_st);

	//ROS_INFO_STREAM("fake odom publishing...");
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "fake_odom_frame");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::Rate rate(50);

	ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu_data", 1, &imuCallback);
	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &odomCallback);

	_stop_publish = false;
	while(!_stop_publish) {
		ros::spinOnce();
		publishTF();
		rate.sleep();
	}

}
