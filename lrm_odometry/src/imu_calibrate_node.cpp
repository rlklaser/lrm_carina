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
 * @file imu_calibrate_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jun 28, 2013
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <boost/thread/mutex.hpp>

#include "lrm_odometry/SetIMUOffset.h"

ros::Publisher imu_pub;
double _offset;
double _drift;
double _time_drift;
ros::Time _start_time;
boost::mutex mutex;

void offsetCallback(std_msgs::Float32::ConstPtr msg)
{
	boost::unique_lock < boost::mutex > scoped_lock(mutex);

	_offset = msg->data;
}

void imuCallback(sensor_msgs::Imu::ConstPtr msg)
{
	if(ros::Time::now()==ros::Time(0)) {
		ROS_INFO_STREAM("time not initialized");
		return;
	}

	boost::unique_lock < boost::mutex > scoped_lock(mutex);

	//ROS_INFO_STREAM("yaw:" << angles::to_degrees(tf::getYaw(msg->orientation)));
	//double yaw = tf::getYaw(msg->orientation);
	tfScalar pitch, roll, yaw;
	tf::Quaternion bt_q;
	tf::quaternionMsgToTF(msg->orientation, bt_q);
	tf::Matrix3x3(bt_q).getRPY(roll, pitch, yaw);

	if(_start_time==ros::Time(0)) _start_time = ros::Time::now();

	int sec = (ros::Time::now() - _start_time).sec;

	_time_drift = _drift * sec;

	sensor_msgs::Imu imu = *msg;
	yaw = angles::normalize_angle(yaw + _offset + _time_drift /*+ angles::from_degrees(90)*/);
	imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

	imu_pub.publish(imu);

	//int int_secs = (int)sec;
	//if(sec % 10 == 0)
	//	ROS_DEBUG_STREAM("time drift:" << angles::to_degrees(_time_drift) << " secs:" << sec << " start:" << _start_time);
	ROS_DEBUG_STREAM_THROTTLE(1, "time drift:" << angles::to_degrees(_time_drift) << " secs:" << sec << " start:" << _start_time);
}

bool offsetService(lrm_odometry::SetIMUOffset::Request& req, lrm_odometry::SetIMUOffset::Response& res)
{
	boost::unique_lock<boost::mutex>scoped_lock(mutex);

	_offset = angles::from_degrees(req.offset);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_calibrate_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	_start_time = ros::Time(0);
	_time_drift = 0;

	imu_pub = nh.advertise<sensor_msgs::Imu>(nh_priv.getNamespace() + "/imu/data", 10);
	ros::ServiceServer service = nh.advertiseService(nh_priv.getNamespace() + "/set_declination", offsetService);
	ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu_data", 1, imuCallback);

	double offset_deg;
	double drift_deg;
	bool fixed_offset;

	nh_priv.param<bool>("fixed_declination", fixed_offset, true);
	nh_priv.param<double>("declination", offset_deg, 0.0);
	nh_priv.param<double>("time_drift", drift_deg, angles::to_degrees(0.0001)); //from xsens datasheet
	_offset = angles::from_degrees(offset_deg);
	_drift = angles::from_degrees(drift_deg);

	ROS_INFO_STREAM("imu calibration node started (offset:" << offset_deg << ")");

	if(!fixed_offset) {
		ros::Subscriber decl_sub = nh.subscribe<std_msgs::Float32>("declination", 1, offsetCallback);
		ROS_WARN_STREAM("imu calibration node subscribed to declination");
	}

	ros::spin();
}
