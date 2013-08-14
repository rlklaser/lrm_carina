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
 * @file position_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Apr 17, 2013
 *
 */

//subscribe odometry and imu
//publish tf
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include <boost/thread/mutex.hpp>

boost::mutex mutex;

void odomCallback(const nav_msgs::Odometry::ConstPtr & msg) {
	boost::unique_lock < boost::mutex > scoped_lock(mutex);
}

void imuCallback(const sensor_msgs::ImuPtr msg_ptr) {
	boost::unique_lock < boost::mutex > scoped_lock(mutex);
	//msg_ptr

}

void publishTF() {
	tf::StampedTransform trans_base_enc;
	try {
		listener.lookupTransform("base_footprint", "imu_link", ros::Time(0), trans_base_enc);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("position_node: %s", ex.what());
		return false;
	}

	tf::Quaternion qt = tf::createQuaternionFromYaw(odo.theta);

	tf::Transform trans_odom_encoder(qt, tf::Vector3(odo.x, odo.y, 0.0));

	tf::StampedTransform trans_odom_base_st(trans_odom_encoder, current_time, // - ros::Duration(0.5), /*trans_base_enc.stamp_,*/
			odo.p.fixed_odometry, odo.p.base_footprint);

	odom_broadcaster.sendTransform(base_to_odom_msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "position_node");
	ros::NodeHandle node;
	ros::NodeHandle priv_node("~");

	//ros::MultiThreadedSpinner spinner(4); // Use 4 threads
	//spinner.spin(); // spin() will not return until the node has been shutdown
	ros::AsyncSpinner spinner(2); // Use 4 threads
	spinner.start();
	ros::waitForShutdown();

}
