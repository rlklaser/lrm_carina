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
 * @file compass_tf_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jul 25, 2013
 *
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

tf::Quaternion _qt;
bool _initialized;

/*
void fixCallback(const sensor_msgs::NavSatFixConstPtr& fix) {
	if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
		ROS_WARN("GPS coordinates still invalid");
		return;
	}

	if (fix->header.stamp == ros::Time(0)) {
		ROS_WARN("Paused");
		return;
	}

	if (!initialized) {
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

		ROS_INFO_STREAM("geographic_to_map_node initialized, lat:" << latitude << " lon:" << longitude << " x:" << pos_x << " y:" << pos_y);
	}
}
*/

int main(int argc, char **argv) {
	ros::init(argc, argv, "compass_tf_node");
	ros::NodeHandle node;
	ros::NodeHandle priv_node("~");
	ros::Subscriber imu_sub;
	tf::TransformBroadcaster tf_broadcaster;
	tf::TransformListener tf_listener(node);

	double rate;
	bool fixed;
	std::string robot_base;
	std::string frame_id;
	std::string fixed_frame;

	priv_node.param<double>("rate", rate, 50.0);
	priv_node.param<std::string>("frame_id", frame_id, "compass_link");
	priv_node.param<std::string>("fixed_frame", fixed_frame, "world");
	priv_node.param<std::string>("robot_base", robot_base, "base_footprint");
	priv_node.param<bool>("fixed", fixed, true);

	_initialized = fixed;

	if (!fixed) {
		//ROS_INFO_STREAM("subscribeing gps");
//		fix_sub = node.subscribe("fix", 10, fixCallback);
	}

	_qt = tf::Quaternion(0, 0, 0, 1);

	ros::Rate step(rate);

	ros::spinOnce();

	while (ros::ok()) {

		ros::spinOnce();

		if (_initialized) {
			tf::Transform transform;
			transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
			if (fixed) {
				tf::StampedTransform movingToFixedTf;
				try {
					tf_listener.waitForTransform(fixed_frame, robot_base, ros::Time(0), ros::Duration(1.0));
					tf_listener.lookupTransform(fixed_frame, robot_base, ros::Time(0), movingToFixedTf);
					_qt = movingToFixedTf.inverse().getRotation();
				} catch (tf::TransformException& ex) {
					ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", looping...");
					continue;
				}
			}
			transform.setRotation(_qt);
			tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), robot_base, frame_id));

			ROS_DEBUG_STREAM("sending transform from (parent)" << robot_base << " to " << frame_id);
		}

		step.sleep();
	}

	return 0;
}
