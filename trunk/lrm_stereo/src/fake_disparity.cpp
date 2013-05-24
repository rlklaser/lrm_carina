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
 * @file fake_disparity.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 2, 2013
 *
 */

#include <ros/ros.h>

class FakeDisparity {

public:

	FakeDisparity(NodeHandle nh) : _nh(nh) {

	};

	void init() {
		pub_disparity_ = nh.advertise<DisparityImage>("disparity", 1, false);


	};

protected:

	void DisparitySGBMNodelet::cameraCallback(const CameraInfoConstPtr& l_info_msg, const CameraInfoConstPtr& r_info_msg) {
		// Update the camera model
		model_.fromCameraInfo(l_info_msg, r_info_msg);

		// Allocate new disparity image message
		DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
		disp_msg->header = l_info_msg->header;
		disp_msg->image.header = l_info_msg->header;
		disp_msg->image.height = l_image_msg->height;
		disp_msg->image.width = l_image_msg->width;
		disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
		disp_msg->image.step = disp_msg->image.width * sizeof(float);
		disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);

		// Stereo parameters
		disp_msg->f = model_.right().fx();
		disp_msg->T = model_.baseline();


		disp_msg->valid_window.x_offset = 0;
		disp_msg->valid_window.y_offset = 0;
		disp_msg->valid_window.width = disp_msg->image.width;
		disp_msg->valid_window.height = disp_msg->image.height;

		// Disparity search range
		disp_msg->min_disparity = 0;
		disp_msg->max_disparity = 16;
		disp_msg->delta_d = 1.0 / 16; // OpenCV uses 16 disparities per pixel

		// Create cv::Mat views onto all buffers
		const cv::Mat_<uint8_t> l_image(l_image_msg->height, l_image_msg->width, const_cast<uint8_t*>(&l_image_msg->data[0]), l_image_msg->step);
		const cv::Mat_<uint8_t> r_image(r_image_msg->height, r_image_msg->width, const_cast<uint8_t*>(&r_image_msg->data[0]), r_image_msg->step);

		cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width, reinterpret_cast<float*>(&disp_msg->image.data[0]), disp_msg->image.step);

		disp_image = 8;

		pub_disparity_.publish(disp_msg);
	}

private:
	NodeHandle _nh;
	ros::Publisher pub_disparity_;
	image_geometry::StereoCameraModel model_;

};
