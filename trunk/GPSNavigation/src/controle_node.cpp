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
 * @file controle_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Mar 29, 2013
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <atuacao/SteeringAngle.h>
#include <controle/Velocity.h>

#include <math.h>

//using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher _controle_pub;

geometry_msgs::Twist twist_;

/*
 void callback(const atuacao::SteeringAnglePtr steer, const controle::VelocityPtr& vel) {

 geometry_msgs::Twist msgs;

 msgs.linear.x = vel->value;
 msgs.angular.z = steer->angle;

 _controle_pub(msgs);
 }
 */

void callback_vel(const controle::VelocityPtr msgs) {
	twist_.linear.x = msgs->value;
	_controle_pub.publish(twist_);
}

void callback_steer(const atuacao::SteeringAnglePtr msgs) {
	twist_.angular.z = -msgs->angle * M_PI / 180;
	_controle_pub.publish(twist_);
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "controle_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	/*
	 message_filters::Subscriber<atuacao::SteeringAngle> steer_sub(nh, "followme/steering_command", 1);
	 message_filters::Subscriber<controle::Velocity> vel_sub(nh, "followme/velocity_command", 1);
	 TimeSynchronizer<atuacao::SteeringAngle, controle::Velocity> sync(steer_sub, vel_sub, 10);
	 sync.registerCallback(boost::bind(&callback, _1, _2));
	 */
	_controle_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Subscriber sub_steer = nh.subscribe("followme/steering_command", 1, callback_steer);
	ros::Subscriber sub_vel = nh.subscribe("followme/velocity_command", 1, callback_vel);

	ROS_INFO_STREAM("conv node start spinning...");

	ros::spin();

	return 0;

}

