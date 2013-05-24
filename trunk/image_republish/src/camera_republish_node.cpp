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
 * @file camera_republish_node.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Sep 27, 2012
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

image_transport::ImageTransport *it_;
sensor_msgs::Image image_;
sensor_msgs::CameraInfo cam_info_;

/** image transport publish interface */
image_transport::CameraPublisher image_pub_;

bool p_restamp;
std::string p_camera_prefix;
std::string p_namespace;

void image_callback(const sensor_msgs::ImagePtr msgs)
{
	image_ = *msgs;

	if(p_restamp) {
		image_.header.stamp = ros::Time::now();
	}
	cam_info_.header.stamp = image_.header.stamp;
	cam_info_.height =image_.height;
	cam_info_.width = image_.width;

	image_pub_.publish(image_, cam_info_);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "republish_camerainfo");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	nh_priv.param("restamp", p_restamp, false);
	nh_priv.param("camera_prefix", p_camera_prefix, std::string("camera"));
	nh_priv.param("namespace", p_namespace, std::string(""));

	it_ = new image_transport::ImageTransport(nh);
	image_pub_ = it_->advertiseCamera(p_namespace + "/" + "image_raw", 1);

	camera_info_manager::CameraInfoManager info_manager(nh_priv, p_camera_prefix);
	cam_info_ = info_manager.getCameraInfo();

    ros::Subscriber sub = nh.subscribe("image_old", 1, image_callback);

    ros::spin();
}
