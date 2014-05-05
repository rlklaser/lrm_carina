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
 * @file fake_encoders_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 4, 2014
 *
 */

#include <ros/ros.h>
#include <lrm_msgs/Encoders.h>
#include <lrm_msgs/Constants.h>
#include <dynamic_reconfigure/server.h>

#include "lrm_drivers/FakeEncodersConfig.h"


int _steer_absolute;
int _wheel_relative;
double _rate;
boost::mutex _mutex;

ros::Publisher _encoders_pub;
lrm_msgs::Encoders msg;

void reconfig(lrm_drivers::FakeEncodersConfig &config, uint32_t level) {

	boost::unique_lock<boost::mutex> scoped_lock(_mutex);

	_rate = config.rate;
	_steer_absolute = config.steer_absolute;
	_wheel_relative = config.wheel_relative;

}

void publish() {

	boost::unique_lock<boost::mutex> scoped_lock(_mutex);

	lrm_msgs::Encoders msg;
	msg.header.stamp = ros::Time::now();
	msg.steering.absolute = _steer_absolute;
	msg.wheel.relative = _wheel_relative;

	_encoders_pub.publish(msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "fake_encoders_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	nh_priv.param("rate", _rate, 10.0);
	nh_priv.param("steer_absolute", _steer_absolute, 0);
	nh_priv.param("wheel_relative", _wheel_relative, 0);

	dynamic_reconfigure::Server<lrm_drivers::FakeEncodersConfig> srv;
	dynamic_reconfigure::Server<lrm_drivers::FakeEncodersConfig>::CallbackType f = boost::bind(&reconfig, _1, _2);
	srv.setCallback(f);

	ros::Rate r(_rate);
	_encoders_pub = nh.advertise<lrm_msgs::Encoders>("encoders", 1);

	while (ros::ok()) {
		publish();
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
