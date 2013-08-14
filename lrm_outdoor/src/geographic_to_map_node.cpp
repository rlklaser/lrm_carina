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
 * @file geographic_to_map_node.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jul 21, 2013
 *
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>

bool initialized;
double pos_x;
double pos_y;

void fixCallback(const sensor_msgs::NavSatFixConstPtr& fix)
{
	if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
	{
		ROS_WARN("GPS coordinates still invalid");
		return;
	}

	if (fix->header.stamp == ros::Time(0))
	{
		ROS_WARN("Paused");
		return;
	}

	if (!initialized)
	{
		initialized = true;
		double latitude = fix->latitude;
		double longitude = fix->longitude;
		//double northing, easting;
		std::string zone;

		geodesy::UTMPoint utm_pt;
		geographic_msgs::GeoPoint geo_pt;
		geometry_msgs::Point point;

		geo_pt = geodesy::toMsg(*fix);
		geo_pt.latitude *= -1;
		geodesy::fromMsg(geo_pt, utm_pt);
		point = geodesy::toGeometry(utm_pt);

		pos_y = -point.y;
		pos_x = point.x;

		ROS_INFO_STREAM("geographic_to_map_node initialized, lat:" <<
				latitude << " lon:" << longitude <<
				" x:" << pos_x << " y:" << pos_y);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "geographic_to_map_node");
	ros::NodeHandle node;
	ros::NodeHandle priv_node("~");
	ros::Subscriber fix_sub;
	tf::TransformBroadcaster tf_broadcaster;
	double rate;
	bool fixed;
	std::string map_frame_id;
	std::string geographic_frame_id;

	priv_node.param<double>("rate", rate, 50.0);
	priv_node.param<std::string>("map_frame_id", map_frame_id, "/map");
	priv_node.param<std::string>("geographic_frame_id", geographic_frame_id, "/geographic_map");
	priv_node.param<bool>("fixed", fixed, false);

	pos_x = 200891;
	pos_y = -2436442;

	initialized = fixed;

	if(!fixed) {
		//ROS_INFO_STREAM("subscribeing gps");
		fix_sub = node.subscribe("fix", 10, fixCallback);
	}

	ros::Rate step(rate);

	while (ros::ok())
	{
		if(initialized) {
			tf::Transform transform;
			transform.setOrigin(tf::Vector3(pos_x, pos_y, 0.0));
			//TODO: probably must correct heading!!!
			transform.setRotation(tf::Quaternion(0, 0, 0, 1));
			tf_broadcaster.sendTransform(
					tf::StampedTransform(transform, ros::Time::now(),
							geographic_frame_id, map_frame_id));

			ROS_DEBUG_STREAM("sending transform from (parent)" << geographic_frame_id << " to " << map_frame_id);
		}
		ros::spinOnce();
		step.sleep();
	}

	return 0;
}
