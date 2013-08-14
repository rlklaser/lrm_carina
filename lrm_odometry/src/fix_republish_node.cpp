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
 * @file fix_republish_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Apr 9, 2013
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

ros::Publisher fix_pub;

void callback(const sensor_msgs::NavSatFixConstPtr& fix) {
	//fix->header.stamp = ros::Time::now();

	sensor_msgs::NavSatFix my_fix = fix;
	fix_pub.publish(my_fix);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "fix_republish_node");
	ros::NodeHandle nh;

	fix_pub = nh.advertise<sensor_msgs::NavSatFix>("old_fix", 10);
	ros::Subscriber fix_sub = nh.subscribe("new_fix", 10, callback);

	ros::spin();
}
