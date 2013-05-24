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
 * @file split_bumblebee.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Feb 1, 2013
 *
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>

#include <boost/make_shared.hpp>

namespace lrm_sensors {

class SplitNodelet: public nodelet::Nodelet {
	sensor_msgs::Image left_img_;
	sensor_msgs::Image right_img_;
	sensor_msgs::CameraInfo left_cam_info_;
	sensor_msgs::CameraInfo right_cam_info_;
	//std_msgs::String calibration_settings_;

	image_transport::CameraPublisher left_cam_pub_;
	image_transport::CameraPublisher right_cam_pub_;

	boost::shared_ptr<image_transport::ImageTransport> it_;
	image_transport::Subscriber sub_raw_;

	std::string p_camera_left_prefix;
	std::string p_camera_right_prefix;

	void publishCam(const sensor_msgs::Image& image);
	bool fillImages(sensor_msgs::Image& left_image, sensor_msgs::Image& right_image, std::string encoding_arg, uint32_t rows_arg, uint32_t cols_arg, uint32_t step_arg, const void* data_arg);

	virtual void onInit();
	void connectCb();
	void imageCb(const sensor_msgs::ImageConstPtr& raw_msg);
	//void configCb(Config &config, uint32_t level);
};

void SplitNodelet::onInit() {
	ros::NodeHandle &nh = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();
	ros::NodeHandle nh_left("~/left");
	ros::NodeHandle nh_right("~/right");

	nh_priv.param("camera_left_prefix", p_camera_left_prefix, std::string("camera_left"));
	nh_priv.param("camera_right_prefix", p_camera_right_prefix, std::string("camera_right"));

	it_.reset(new image_transport::ImageTransport(nh));
	left_cam_pub_ = it_->advertiseCamera("/left/image_raw", 1);
	right_cam_pub_ = it_->advertiseCamera("/right/image_raw", 1);

	camera_info_manager::CameraInfoManager info_manager_left(nh_left, p_camera_left_prefix);
	left_cam_info_ = info_manager_left.getCameraInfo();

	camera_info_manager::CameraInfoManager info_manager_right(nh_right, p_camera_right_prefix);
	right_cam_info_ = info_manager_right.getCameraInfo();

	//ros::Subscriber sub = nh.subscribe("image_raw", 1, &imageCb);
	image_transport::TransportHints hints("raw", ros::TransportHints(), nh_priv);
	sub_raw_ = it_->subscribe("image_raw", 1, &SplitNodelet::imageCb, this, hints);
}

void SplitNodelet::connectCb() {
}

void SplitNodelet::imageCb(const sensor_msgs::ImageConstPtr& raw_msg) {
	publishCam(*raw_msg);
}

bool SplitNodelet::fillImages(sensor_msgs::Image& left_image, sensor_msgs::Image& right_image, std::string encoding_arg, uint32_t rows_arg, uint32_t cols_arg, uint32_t step_arg, const void* data_arg) {
	size_t st0 = (step_arg * rows_arg);
	left_image.encoding = encoding_arg;
	left_image.height = rows_arg;
	left_image.width = cols_arg;
	left_image.step = cols_arg; //step_arg;
	left_image.data.resize(st0);

	right_image.encoding = encoding_arg;
	right_image.height = rows_arg;
	right_image.width = cols_arg;
	right_image.step = cols_arg; //step_arg;
	right_image.data.resize(st0);

	char* crt = (char*) data_arg;
	char* left = (char*) &left_image.data[0];
	char* right = (char*) &right_image.data[0];

	int k = 320 * 240;

	//NO DEBAYERING!!!
	while (k > 0) {
		*left = *crt;
		*right = *(crt + 1);

		crt += 2;
		right++;
		left++;

		*left = *crt;
		*right = *(crt + 1);

		crt += 2;
		right++;
		left++;

		*left = *crt;
		*right = *(crt + 1);

		crt += 2;
		right++;
		left++;

		*left = *crt;
		*right = *(crt + 1);

		crt += 2;
		right++;
		left++;

		k--;
	}

	left_image.is_bigendian = 0;
	right_image.is_bigendian = 0;
	return true;
}

void SplitNodelet::publishCam(const sensor_msgs::Image& image) {
	uint32_t imHeight = image.height;
	uint32_t imWidth = image.width;
	uint32_t imStep = imWidth;
	const void *imRaw = image.data.data();

	imStep = 480 * 4;

	std::string encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
	//std::string encoding = sensor_msgs::image_encodings::RGB8;

	fillImages(left_img_, right_img_, encoding, imHeight, imWidth, imStep, imRaw);
	//fillImagesRGB(left_img_, right_img_, encoding, imHeight, imWidth, imStep, imRaw);

	// Update diagnostics and publish
	ros::Time stamp = ros::Time::now();

	left_img_.header.frame_id = "/camera";
	right_img_.header.frame_id = "/camera";
	left_cam_info_.header.frame_id = left_img_.header.frame_id;
	left_cam_info_.header.stamp = stamp;

	left_cam_info_.height = imHeight;
	left_cam_info_.width = imWidth;

	right_cam_info_.header.frame_id = right_img_.header.frame_id;
	right_cam_info_.header.stamp = stamp;

	right_cam_info_.height = imHeight;
	right_cam_info_.width = imWidth;

	left_cam_pub_.publish(left_img_, left_cam_info_, stamp);
	right_cam_pub_.publish(right_img_, right_cam_info_, stamp);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lrm_sensors::SplitNodelet, nodelet::Nodelet)
