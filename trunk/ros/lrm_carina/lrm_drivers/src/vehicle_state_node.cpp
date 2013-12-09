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
 * @file vehicle_state_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Aug 14, 2013
 *
 */

#include <ros/ros.h>
#include <lrm_msgs/Encoders.h>
#include <lrm_msgs/VehicleState.h>
#include <lrm_msgs/Constants.h>

double tread_lenght;
double wheel_encoder;
lrm_msgs::VehicleState state;
ros::Time last_time = ros::Time(0);
ros::Time now = ros::Time(0);
ros::Publisher states_pub;
bool started;
//int i;

void encodersCallback(const lrm_msgs::Encoders::ConstPtr& encoder) {
	wheel_encoder += encoder->wheel.relative;
	now = encoder->header.stamp;
	started = true;
	//encoder->steering.absolute;
	//i++;
	/*
	 ros::Time now = ros::Time::now();
	 ros::Duration dt = now - last_time;
	 last_time = now;
	 static double dist = (tread_lenght / WHEEL_ENCODER_RESOLUTION) * wheel_encoder;
	 static double vel = dist / dt.toSec();
	 state.header.stamp = now;
	 state.velocity = vel * 3.6; //km/h
	 states_pub.publish(state);
	 */
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "vehicle_state_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	double rate;
	double dist;
	double vel;

	nh_priv.param("rate", rate, 5.0);
	nh_priv.param("tread_lenght", tread_lenght, 1.508);

	wheel_encoder = 0;
	ros::Rate r(rate);

	state.header.frame_id = "base_footprint";

	started = false;
	ros::Subscriber encoders_sub = nh.subscribe<lrm_msgs::Encoders>("encoders", 50, &encodersCallback);
	states_pub = nh.advertise<lrm_msgs::VehicleState>("vehicle_state", 1);

	//publisher_timer = n.createTimer(ros::Duration(1.0 / odo.p.rate), &WheelOdometry::odomCallback, this, false, false);

	while (ros::ok()) {
		if(!started) {
			now = ros::Time::now();
			ROS_WARN_STREAM_THROTTLE(1, "wheel encoder not received");
		}
		ros::Duration dt = now - last_time;
		dist = (tread_lenght / WHEEL_ENCODER_RESOLUTION) * wheel_encoder;

		double delta = dt.toSec();
		if (delta > 0) {
			vel = dist / delta;
			wheel_encoder = 0;
			last_time = now;

			state.header.stamp = ros::Time::now(); //now;
			state.velocity = vel; // * 3.6; //km/h
			//i=0;
			states_pub.publish(state);
		}

		ros::spinOnce();
		r.sleep();
	}

	//ros::spin();

	return 0;
}
