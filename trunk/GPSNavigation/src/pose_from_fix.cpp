/*
 *  Copyright (C) 2012-2013, Laboratorio de Robotica Movel - ICMC/USP
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
 * @file pose_from_fix.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Mar 31, 2013
 *
 */
/*
 void callback()
 {
 geometry_msgs::PoseStamped pose;

 pose.header.stamp = now;
 pose.header.frame_id = mFrameID.c_str();

 double northing, easting;
 std::string zone;

 gps_common::LLtoUTM(latitude(), longitude(), northing, easting, zone);
 pose.pose.position.x = easting;
 pose.pose.position.y = northing;
 pose.pose.position.z = altitude();

 if (output_settings.orientationMode == EulerAngles)
 {

 poseEuler.pose.orientation.w = 1.0f;
 poseEuler.pose.orientation.x = roll();
 poseEuler.pose.orientation.y = pitch();
 poseEuler.pose.orientation.z = yaw();

 if (poseEuler.pose.orientation.x < 0.0f)
 {
 poseEuler.pose.orientation.x = 360 + poseEuler.pose.orientation.x;
 }

 if (poseEuler.pose.orientation.y < 0.0f)
 {
 poseEuler.pose.orientation.y = 360 + poseEuler.pose.orientation.y;
 }

 if (poseEuler.pose.orientation.z < 0.0f)
 {
 poseEuler.pose.orientation.z = 360 + poseEuler.pose.orientation.z;
 }
 }
 else
 {
 poseEuler.pose.orientation.x = quaternion_x();
 poseEuler.pose.orientation.y = quaternion_y();
 poseEuler.pose.orientation.z = quaternion_z();
 poseEuler.pose.orientation.w = quaternion_w();
 }

 return poseEuler;
 }
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher _pose_pub;

void callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
	double northing, easting;
	std::string zone;

	geometry_msgs::PoseStamped pose;

	pose.header.stamp = msg->header.stamp;
	pose.header.frame_id = msg->header.frame_id;

	gps_common::LLtoUTM(msg->latitude, msg->longitude, northing, easting, zone);
	pose.pose.position.x = easting;
	pose.pose.position.y = northing;
	pose.pose.position.z = msg->altitude;

	pose.pose.orientation.x = 1;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = 0;
	pose.pose.orientation.w = 0;

	_pose_pub.publish(pose);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pose_from_fix");
	ros::NodeHandle nh;
	ros::Subscriber gps_sub = nh.subscribe("fix", 1, callback);
	_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
	ros::spin();
	return 0;
}
