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
 * @file split_ladybug.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Aug 1, 2013
 *
 */

#include <string.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/make_shared.hpp>

using namespace cv_bridge;
using namespace cv;

#define LB_WIDTH	512
#define LB_HEIGHT	384
#define NUM_IMGS    6

#include <lrm_sensors/split_node.h>

namespace lrm_sensors {

class SplitLadybugNodelet: public nodelet::Nodelet {

	image_transport::Subscriber sub_;
	boost::shared_ptr<image_transport::ImageTransport> it_;

	image_transport::Publisher pub[NUM_IMGS];
	//int seq = 0;

	virtual void onInit();
	//void connectCb();
	void imageCb(const sensor_msgs::ImageConstPtr& raw_msg);
};

void SplitLadybugNodelet::onInit() {

	ros::NodeHandle &nh = getNodeHandle();
	ros::NodeHandle &nh_priv = getPrivateNodeHandle();

	it_.reset(new image_transport::ImageTransport(nh));

	image_transport::TransportHints hints("raw", ros::TransportHints(), nh);
	sub_ = it_->subscribe("/camera/image_raw", 1, &SplitLadybugNodelet::imageCb, this, hints);

	pub[0] = it_->advertise("/camera/image_color/0", 1);
	pub[1] = it_->advertise("/camera/image_color/1", 1);
	pub[2] = it_->advertise("/camera/image_color/2", 1);
	pub[3] = it_->advertise("/camera/image_color/3", 1);
	pub[4] = it_->advertise("/camera/image_color/4", 1);
	pub[5] = it_->advertise("/camera/image_color/5", 1);

}

void SplitLadybugNodelet::imageCb(const sensor_msgs::ImageConstPtr& msg) {
	bool have_subsc = false;
	for (int i = 0; i < NUM_IMGS; i++) {
		if (pub[i].getNumSubscribers() > 0) {
			have_subsc = true;
			break;
		}
	}
	if (have_subsc) {
		try {
			cv_bridge::CvImageConstPtr cv_ptr;
			cv_ptr = toCvShare(msg, "mono8");
			int sz = 0;

			sensor_msgs::Image omsg;
			omsg.header.frame_id = "/ladybug";
			omsg.header.seq = msg->header.seq;
			omsg.header.stamp = msg->header.stamp;

			for (int i = 0; i < NUM_IMGS; i++) {

				const cv::Mat_<uint8_t> img_0(LB_HEIGHT, LB_WIDTH, const_cast<uint8_t*>(&msg->data[sz]), msg->step);
				sz += (LB_WIDTH * LB_HEIGHT);
				const cv::Mat_<uint8_t> img_1(LB_HEIGHT, LB_WIDTH, const_cast<uint8_t*>(&msg->data[sz]), msg->step);
				sz += (LB_WIDTH * LB_HEIGHT);
				const cv::Mat_<uint8_t> img_2(LB_HEIGHT, LB_WIDTH, const_cast<uint8_t*>(&msg->data[sz]), msg->step);
				sz += (LB_WIDTH * LB_HEIGHT);
				const cv::Mat_<uint8_t> img_3(LB_HEIGHT, LB_WIDTH, const_cast<uint8_t*>(&msg->data[sz]), msg->step);
				sz += (LB_WIDTH * LB_HEIGHT);

				if (pub[i].getNumSubscribers() > 0) {
					omsg.width = LB_HEIGHT;
					omsg.height = LB_WIDTH;
					omsg.encoding = "rgb8";
					omsg.is_bigendian = false;
					omsg.step = omsg.width * 3;
					omsg.data.resize(LB_WIDTH * LB_HEIGHT * 3);
					CvImagePtr cp = toCvCopy(omsg, "rgb8");
					Mat src[] = { img_3, img_1, img_0 };
					cv::merge(src, 3, cp->image);
					pub[i].publish(cp->toImageMsg());
				}
			}
		} catch (cv_bridge::Exception& e) {
			//ROS_ERROR("error:" << e);
		}
	}
}

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lrm_sensors::SplitLadybugNodelet, nodelet::Nodelet)
