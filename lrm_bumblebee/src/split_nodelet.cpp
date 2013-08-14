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
 * @file split_nodelet.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Oct 5, 2012
 *
 */

#include "split_nodelet.h"

SplitNodelet::SplitNodelet() {
}

SplitNodelet::SplitNodelet(ros::NodeHandle nh) :
		nh_(nh) {
}

SplitNodelet::~SplitNodelet() {
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
	//char* end = crt + 3*st0;
	char* left = (char*) &left_image.data[0];
	char* right = (char*) &right_image.data[0];

	int k = 320 * 240;

	while (k > 0) {

		//std::cout << k << std::endl;

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

	fillImages(left_img_, right_img_, encoding, imHeight, imWidth, imStep, imRaw);

	// Copy camera info into ROS message
	left_cam_info_.height = imHeight;
	left_cam_info_.width = imWidth;

	right_cam_info_.height = imHeight;
	right_cam_info_.width = imWidth;

	// Update diagnostics and publish
	ros::Time stamp = ros::Time::now();

	left_img_.header.frame_id = "/stereo_camera";
	right_img_.header.frame_id = "/stereo_camera";
	left_cam_info_.header.frame_id = left_img_.header.frame_id;
	right_cam_info_.header.frame_id = right_img_.header.frame_id;


	left_cam_pub_.publish(left_img_, left_cam_info_, stamp);
	right_cam_pub_.publish(right_img_, right_cam_info_, stamp);
}

void SplitNodelet::callback(const sensor_msgs::ImageConstPtr& msgs) {
	publishCam(*msgs);
}

void SplitNodelet::onInit() {
	ros::NodeHandle nh_priv("~");

	image_transport::ImageTransport it(nh_);
	left_cam_pub_ = it.advertiseCamera(nh_priv.getNamespace() + "/left/image_raw", 1);
	right_cam_pub_ = it.advertiseCamera(nh_priv.getNamespace() + "/right/image_raw", 1);

	ros::Subscriber sub = nh_.subscribe("image_raw", 1, &callback);
}

PLUGINLIB_DECLARE_CLASS(lrm_bumblebee, split, SplitNodelet, nodelet::Nodelet)
;
