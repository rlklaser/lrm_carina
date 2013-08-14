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
 * @file pose_velocity_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Apr 24, 2013
 *
 */

#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

using namespace std;

double _rate = 1;
double _x = 0;
double _y = 0;
ros::Publisher _vel_pub;
ros::Time _t(0);

std_msgs::Float32 _msg;

void callback_pose(const geometry_msgs::PoseStampedPtr pose)
{
	//if(_t.isValid()) {

		ros::Duration t = pose->header.stamp - _t;

		if(t.toSec()>_rate) {
			double x = pose->pose.position.x - _x;
			double y = pose->pose.position.y - _y;

			double dist = sqrt(x*x + y*y);

			double vel_ms = dist/t.toSec() ;

			//ROS_DEBUG_STREAM(vel_ms << " m/s (" << vel_ms*3.6 << " km/h)");
			ROS_INFO_STREAM(vel_ms << " m/s (" << vel_ms*3.6 << " km/h)");

			_msg.data = vel_ms;
			_vel_pub.publish(_msg);

			_t = pose->header.stamp;
			_x = pose->pose.position.x;
			_y = pose->pose.position.y;
		}
		//running a bag in a loop, the time goes back
		else if (t.toSec()<0) {
			_t = pose->header.stamp;
		}
	//}
	//else {
	//	ROS_INFO_STREAM("invalid " << _t);
	//	_t = pose->header.stamp;
	//}
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "pose_velocity_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    nh_priv.param("rate", _rate, 0.5);

    ros::Subscriber sub = nh.subscribe("pose2d", 1, callback_pose);
    _vel_pub = nh.advertise<std_msgs::Float32>(nh_priv.getNamespace() + "/velocity", 1);

    ROS_INFO_STREAM("pose velocity start spinning...");

    ros::spin();

    return 0;
}
