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
 * @file split_bb2_node.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jul 28, 2013
 *
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>

#include <lrm_sensors/split_node.h>

#include <dc1394/dc1394.h>
#include <cstring>

namespace lrm_sensors {

class SplitBB2Nodelet: public nodelet::Nodelet {

	struct st_stereo_camera ctx_;

	image_transport::Subscriber sub_;
	boost::shared_ptr<image_transport::ImageTransport> it_;

	void publishCam(const sensor_msgs::Image& image);

	bool fillImages(sensor_msgs::Image& left_image, sensor_msgs::Image& right_image, std::string encoding_arg, uint32_t rows_arg, uint32_t cols_arg, const void* data_arg);

	virtual void onInit();
	void connectCb();
	void imageCb(const sensor_msgs::ImageConstPtr& raw_msg);
};

void SplitBB2Nodelet::onInit() {
	ros::NodeHandle &nh = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	it_.reset(new image_transport::ImageTransport(nh));

	nh_priv.param("name", ctx_.name, std::string("camera"));
	nh_priv.param("frame_id", ctx_.frame_id, std::string("stereo_camera"));
	nh_priv.param("left/camera_info_url", ctx_.left.url, std::string(""));
	nh_priv.param("right/camera_info_url", ctx_.right.url, std::string(""));

	ros::NodeHandle nh_left(ctx_.frame_id + "/left");
	ros::NodeHandle nh_right(ctx_.frame_id + "/right");

	ctx_.left.publisher = it_->advertiseCamera(nh.getNamespace() + "/left/image_raw", 1);
	ctx_.right.publisher = it_->advertiseCamera(nh.getNamespace() + "/right/image_raw", 1);

	ctx_.left.manager.reset(new camera_info_manager::CameraInfoManager(nh_left, ctx_.name + "_left", ctx_.left.url));
	//ctx_.left.info = ctx_.left.manager->getCameraInfo();

	ctx_.right.manager.reset(new camera_info_manager::CameraInfoManager(nh_right, ctx_.name + "_right", ctx_.right.url));
	//ctx_.right.info = ctx_.right.manager->getCameraInfo();

	image_transport::TransportHints hints("raw", ros::TransportHints(), nh);
	sub_ = it_->subscribe("image_raw", 1, &SplitBB2Nodelet::imageCb, this, hints);
}

void SplitBB2Nodelet::connectCb() {
}

void SplitBB2Nodelet::imageCb(const sensor_msgs::ImageConstPtr& raw_msg) {
	publishCam(*raw_msg);
}

bool SplitBB2Nodelet::fillImages(sensor_msgs::Image& left_image, sensor_msgs::Image& right_image, std::string encoding_arg, uint32_t rows_arg, uint32_t cols_arg, const void* data_arg) {

	uint32_t step_arg = cols_arg * sensor_msgs::image_encodings::numChannels(encoding_arg);

	size_t st0 = (step_arg * rows_arg);
	left_image.encoding = encoding_arg;
	left_image.height = rows_arg;
	left_image.width = cols_arg;
	left_image.step = cols_arg;
	left_image.data.resize(st0); //*2

	right_image.encoding = encoding_arg;
	right_image.height = rows_arg;
	right_image.width = cols_arg;
	right_image.step = cols_arg;
	right_image.data.resize(st0);

	uint8_t* crt = (uint8_t*) data_arg;
	uint8_t* left = (uint8_t*) &left_image.data[0];
	uint8_t* right = (uint8_t*) &right_image.data[0];

	int k = (cols_arg * rows_arg);

	//dc1394_deinterlace_stereo(crt, left, cols_arg, rows_arg);
	//memcpy(right, left+k, k);

	//deinterlace stereo
	while (k > 0) {
		*(left++) = *(crt++);
		*(right++) = *(crt++);
		k--;
	}

	left_image.is_bigendian = 0;
	right_image.is_bigendian = 0;
	return true;
}

void SplitBB2Nodelet::publishCam(const sensor_msgs::Image& image) {
	uint32_t imHeight = image.height;
	uint32_t imWidth = image.width;
	const void *imRaw = image.data.data();

	std::string encoding = sensor_msgs::image_encodings::BAYER_BGGR8;

	fillImages(ctx_.left.image, ctx_.right.image, encoding, imHeight, imWidth, imRaw);

	// Update diagnostics and publish
	ros::Time stamp = image.header.stamp;

	ctx_.frame_id = ctx_.frame_id == "" ? image.header.frame_id : ctx_.frame_id;

	ctx_.left.image.header.frame_id = ctx_.frame_id;
	ctx_.right.image.header.frame_id = ctx_.frame_id;

	ctx_.left.info = ctx_.left.manager->getCameraInfo();
	ctx_.left.info.header.frame_id = ctx_.frame_id;
	ctx_.left.info.header.stamp = stamp;
	ctx_.left.info.height = imHeight;
	ctx_.left.info.width = imWidth;

	ctx_.right.info = ctx_.right.manager->getCameraInfo();
	ctx_.right.info.header.frame_id = ctx_.frame_id;
	ctx_.right.info.header.stamp = stamp;
	ctx_.right.info.height = imHeight;
	ctx_.right.info.width = imWidth;

	ctx_.left.publisher.publish(ctx_.left.image, ctx_.left.info, stamp);
	ctx_.right.publisher.publish(ctx_.right.image, ctx_.right.info, stamp);
}

}
;

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lrm_sensors::SplitBB2Nodelet, nodelet::Nodelet)
