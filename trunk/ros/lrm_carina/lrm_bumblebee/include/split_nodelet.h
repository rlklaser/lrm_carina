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
 * @file split_nodelet.h
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Oct 5, 2012
 *
 */

#ifndef SPLIT_NODELET_H_
#define SPLIT_NODELET_H_


#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

class SplitNodelet: public nodelet::Nodelet
{
public:
	SplitNodelet();
    SplitNodelet(ros::NodeHandle);
	virtual ~SplitNodelet();

	virtual void onInit();

private:
	sensor_msgs::Image        left_img_;
	sensor_msgs::Image        right_img_;
	sensor_msgs::CameraInfo   left_cam_info_;
	sensor_msgs::CameraInfo   right_cam_info_;
	std_msgs::String		  calibration_settings_;

	image_transport::CameraPublisher left_cam_pub_;
	image_transport::CameraPublisher right_cam_pub_;

	ros::NodeHandle nh_;

	bool fillImages(sensor_msgs::Image& left_image, sensor_msgs::Image& right_image,
			std::string encoding_arg,
			uint32_t rows_arg,
			uint32_t cols_arg,
			uint32_t step_arg,
			const void* data_arg);

	void publishCam(const sensor_msgs::Image& image);

	void callback(const sensor_msgs::ImageConstPtr& msgs);
};


#endif /* SPLIT_NODELET_H_ */
