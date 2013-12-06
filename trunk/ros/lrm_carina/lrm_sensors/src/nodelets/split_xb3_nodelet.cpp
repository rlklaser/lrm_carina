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
 * @file split_xb3_node.cpp
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

namespace lrm_sensors {

class SplitXB3Nodelet: public nodelet::Nodelet {

	struct st_stereo_camera wide_ctx_;
	struct st_stereo_camera narrow_ctx_;

	image_transport::Subscriber sub_;
	boost::shared_ptr<image_transport::ImageTransport> it_;

	void publishCam(const sensor_msgs::Image& image);

	bool fillImages(sensor_msgs::Image& narrow_left_image, sensor_msgs::Image&
			narrow_right_image, sensor_msgs::Image& wide_left_image,
			sensor_msgs::Image& wide_right_image, std::string encoding_arg,
			uint32_t rows_arg, uint32_t cols_arg, const void* data_arg);

	virtual void onInit();
	void connectCb();
	void imageCb(const sensor_msgs::ImageConstPtr& raw_msg);
};

void SplitXB3Nodelet::onInit() {
	ros::NodeHandle &nh = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	it_.reset(new image_transport::ImageTransport(nh));

	std::string name;
	std::string narrow_frame_id;
	std::string wide_frame_id;

	nh_priv.param("name", name, std::string("camera"));
	nh_priv.param("narrow/frame_id", narrow_ctx_.frame_id, std::string(""));
	nh_priv.param("narrow/left/camera_info_url", narrow_ctx_.left.url, std::string(""));
	nh_priv.param("narrow/right/camera_info_url", narrow_ctx_.right.url, std::string(""));

	narrow_ctx_.name = name;
	wide_ctx_.name = name;

	{ //narrow
		ros::NodeHandle nh_baseline(nh.getNamespace() + "/narrow");
		ros::NodeHandle nh_left(nh_baseline.getNamespace() + "/left");
		ros::NodeHandle nh_right(nh_baseline.getNamespace() + "/right");

		narrow_ctx_.left.publisher = it_->advertiseCamera(nh_left.getNamespace() + "/image_raw", 1);
		narrow_ctx_.right.publisher = it_->advertiseCamera(nh_right.getNamespace() + "/image_raw", 1);

		narrow_ctx_.left.manager.reset(new camera_info_manager::CameraInfoManager(
				nh_left, narrow_ctx_.name + "_narrow_left", narrow_ctx_.left.url));
		narrow_ctx_.right.manager.reset(new camera_info_manager::CameraInfoManager(
				nh_right, narrow_ctx_.name + "_narrow_right", narrow_ctx_.right.url));
	}

	{ //wide
		ros::NodeHandle nh_baseline(nh.getNamespace() + "/wide");
		ros::NodeHandle nh_left(nh_baseline.getNamespace() + "/left");
		ros::NodeHandle nh_right(nh_baseline.getNamespace() + "/right");

		nh_priv.param("wide/left/camera_info_url", wide_ctx_.left.url, std::string(""));
		nh_priv.param("wide/right/camera_info_url", wide_ctx_.right.url, std::string(""));

		wide_ctx_.left.publisher = it_->advertiseCamera(nh_left.getNamespace() + "/image_raw", 1);
		wide_ctx_.right.publisher = it_->advertiseCamera(nh_right.getNamespace() + "/image_raw", 1);

		wide_ctx_.left.manager.reset(new camera_info_manager::CameraInfoManager(
				nh_left, wide_ctx_.name + "_wide_left", wide_ctx_.left.url));
		wide_ctx_.right.manager.reset(new camera_info_manager::CameraInfoManager(
				nh_right, wide_ctx_.name + "_wide_right", wide_ctx_.right.url));
	}

	image_transport::TransportHints hints("raw", ros::TransportHints(), nh);
	sub_ = it_->subscribe("image_raw", 1, &SplitXB3Nodelet::imageCb, this, hints);
}

void SplitXB3Nodelet::connectCb() {
}

void SplitXB3Nodelet::imageCb(const sensor_msgs::ImageConstPtr& raw_msg) {
	publishCam(*raw_msg);
}

bool SplitXB3Nodelet::fillImages(sensor_msgs::Image& narrow_left_image, sensor_msgs::Image&
		narrow_right_image, sensor_msgs::Image& wide_left_image, sensor_msgs::Image& wide_right_image,
		std::string encoding_arg, uint32_t rows_arg, uint32_t cols_arg, const void* data_arg) {

	uint32_t step_arg = cols_arg * sensor_msgs::image_encodings::numChannels(encoding_arg);

	size_t st0 = (step_arg * rows_arg);

	narrow_left_image.encoding = encoding_arg;
	narrow_left_image.height = rows_arg;// / 2;
	narrow_left_image.width = cols_arg;// / 2;
	narrow_left_image.step = cols_arg;
	narrow_left_image.data.resize(st0);

	narrow_right_image.encoding = encoding_arg;
	narrow_right_image.height = rows_arg;// / 2;
	narrow_right_image.width = cols_arg;// / 2;
	narrow_right_image.step = cols_arg;
	narrow_right_image.data.resize(st0);

	wide_left_image.encoding = encoding_arg;
	wide_left_image.height = rows_arg;// / 2;
	wide_left_image.width = cols_arg;// / 2;
	wide_left_image.step = cols_arg;
	wide_left_image.data.resize(st0);

	wide_right_image.encoding = encoding_arg;
	wide_right_image.height = rows_arg;// / 2;
	wide_right_image.width = cols_arg;// / 2;
	wide_right_image.step = cols_arg;
	wide_right_image.data.resize(st0);

	char* crt = (char*) data_arg;
	char* narrow_left = (char*) &narrow_left_image.data[0];
	char* narrow_right = (char*) &narrow_right_image.data[0];
	char* wide_left = (char*) &wide_left_image.data[0];
	char* wide_right = (char*) &wide_right_image.data[0];

	int k = cols_arg * rows_arg;

	//deinterlace
	while (k > 0) {
		/*
		*(narrow_right++) = *crt;
		*(wide_right++) = *(crt++);
		*(narrow_left++) = *(crt++);
		*(wide_left++) = *(crt++);
		*/
		*(wide_right++) = *(crt++);
		*(narrow_right++) = *(crt++);
		*(narrow_left++) = *(crt);
		*(wide_left++) = *(crt++);
		k--;
	}

	narrow_left_image.is_bigendian = 0;
	narrow_right_image.is_bigendian = 0;
	wide_left_image.is_bigendian = 0;
	wide_right_image.is_bigendian = 0;

	return true;
}

void SplitXB3Nodelet::publishCam(const sensor_msgs::Image& image) {
	uint32_t imHeight = image.height;
	uint32_t imWidth = image.width;
	const void *imRaw = image.data.data();

	std::string encoding = /*sensor_msgs::image_encodings::MONO8;*/ sensor_msgs::image_encodings::BAYER_GBRG8;

	fillImages(narrow_ctx_.left.image, narrow_ctx_.right.image,
			wide_ctx_.left.image, wide_ctx_.right.image,
			encoding, imHeight, imWidth, imRaw);

	// Update diagnostics and publish
	ros::Time stamp = image.header.stamp;

	//narrow
	narrow_ctx_.frame_id = narrow_ctx_.frame_id == "" ? image.header.frame_id : narrow_ctx_.frame_id;

	narrow_ctx_.left.image.header.frame_id = narrow_ctx_.frame_id;
	//narrow_ctx_.left.image.header.stamp = stamp;
	//narrow_ctx_.left.image.encoding = sensor_msgs::image_encodings::BAYER_GBRG8;

	narrow_ctx_.right.image.header.frame_id = narrow_ctx_.frame_id;

	narrow_ctx_.left.info = narrow_ctx_.left.manager->getCameraInfo();
	narrow_ctx_.left.info.header.frame_id = narrow_ctx_.frame_id;
	//narrow_ctx_.left.info.header.stamp = stamp;
	narrow_ctx_.left.info.height = imHeight;
	narrow_ctx_.left.info.width = imWidth;

	narrow_ctx_.right.info = narrow_ctx_.right.manager->getCameraInfo();
	narrow_ctx_.right.info.header.frame_id = narrow_ctx_.frame_id;
	//narrow_ctx_.right.info.header.stamp = stamp;
	narrow_ctx_.right.info.height = imHeight;
	narrow_ctx_.right.info.width = imWidth;

	narrow_ctx_.left.publisher.publish(narrow_ctx_.left.image, narrow_ctx_.left.info, stamp);
	narrow_ctx_.right.publisher.publish(narrow_ctx_.right.image, narrow_ctx_.right.info, stamp);

	//wide
	wide_ctx_.frame_id = wide_ctx_.frame_id == "" ? image.header.frame_id : wide_ctx_.frame_id;

	wide_ctx_.left.image.header.frame_id = wide_ctx_.frame_id;
	wide_ctx_.right.image.header.frame_id = wide_ctx_.frame_id;

	wide_ctx_.left.info = wide_ctx_.left.manager->getCameraInfo();
	wide_ctx_.left.info.header.frame_id = wide_ctx_.frame_id;
	//wide_ctx_.left.info.header.stamp = stamp;
	wide_ctx_.left.info.height = imHeight;
	wide_ctx_.left.info.width = imWidth;

	wide_ctx_.right.info = wide_ctx_.right.manager->getCameraInfo();
	wide_ctx_.right.info.header.frame_id = wide_ctx_.frame_id;
	//wide_ctx_.right.info.header.stamp = stamp;
	wide_ctx_.right.info.height = imHeight;
	wide_ctx_.right.info.width = imWidth;

	wide_ctx_.left.publisher.publish(wide_ctx_.left.image, wide_ctx_.left.info, stamp);
	wide_ctx_.right.publisher.publish(wide_ctx_.right.image, wide_ctx_.right.info, stamp);
}

}
;

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lrm_sensors::SplitXB3Nodelet, nodelet::Nodelet)
