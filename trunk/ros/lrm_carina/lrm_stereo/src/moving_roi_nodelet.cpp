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
 * @file moving_roi_nodelet.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 20, 2013
 *
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pluginlib/class_list_macros.h>
#include <image_geometry/stereo_camera_model.h>

namespace lrm_stereo {

using namespace sensor_msgs;
using namespace message_filters::sync_policies;

typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

class MovingROINodelet: public nodelet::Nodelet {

	// Subscriptions
	image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
	message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;

	boost::shared_ptr<ExactSync> exact_sync_;
	boost::shared_ptr<ApproximateSync> approximate_sync_;
	boost::shared_ptr<image_transport::ImageTransport> it_;

	CameraInfo left_cam_info_;
	CameraInfo right_cam_info_;

	ros::Publisher left_image_pub_;
	ros::Publisher right_image_pub_;
	ros::Publisher left_camera_info_pub_;
	ros::Publisher right_camera_info_pub_;

// Dynamic reconfigure
//	boost::recursive_mutex config_mutex_;
//	typedef lrm_stereo::DisparitySGBMConfig Config;
//	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
//	boost::shared_ptr<ReconfigureServer> reconfigure_server_;

// Processing state (note: only safe because we're single-threaded!)
	image_geometry::StereoCameraModel model_;

	virtual void onInit();
	void imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg, const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg);
};

void MovingROINodelet::onInit() {
	ros::NodeHandle &nh = getNodeHandle();
	ros::NodeHandle &private_nh = getPrivateNodeHandle();

	// Synchronize inputs. Topic subscriptions happen on demand in the connection
	// callback. Optionally do approximate synchronization.
	int queue_size;
	private_nh.param("queue_size", queue_size, 5);
	bool approx;
	private_nh.param("approximate_sync", approx, true);
	std::string out_topic;
	private_nh.param<std::string>("output_namespace", out_topic, "stereo_roi");

	if (approx) {
		approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size), sub_l_image_, sub_l_info_, sub_r_image_, sub_r_info_));
		approximate_sync_->registerCallback(boost::bind(&MovingROINodelet::imageCb, this, _1, _2, _3, _4));
	} else {
		exact_sync_.reset(new ExactSync(ExactPolicy(queue_size), sub_l_image_, sub_l_info_, sub_r_image_, sub_r_info_));
		exact_sync_->registerCallback(boost::bind(&MovingROINodelet::imageCb, this, _1, _2, _3, _4));
	}

	//Set up dynamic reconfiguration
//	ReconfigureServer::CallbackType f = boost::bind(&DisparitySGBMNodelet::configCb, this, _1, _2);
//	reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
//	reconfigure_server_->setCallback(f);

	//broadcast the throttled versions on out_topic
	left_image_pub_ = nh.advertise<Image>(out_topic + std::string("/left/image_rect"), 2);
	right_image_pub_ = nh.advertise<Image>(out_topic + std::string("/right/image_rect"), 2);
	left_camera_info_pub_ = nh.advertise<CameraInfo>(out_topic + std::string("/left/camera_info"), 2);
	right_camera_info_pub_ = nh.advertise<CameraInfo>(out_topic + std::string("/right/camera_info"), 2);

	image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
	it_.reset(new image_transport::ImageTransport(nh));
	sub_l_image_.subscribe(*it_, "left/image_rect", 1, hints);
	sub_l_info_.subscribe(nh, "left/camera_info", 1);
	sub_r_image_.subscribe(*it_, "right/image_rect", 1, hints);
	sub_r_info_.subscribe(nh, "right/camera_info", 1);

	ROS_WARN_STREAM("moving roi initialized.");
}

void MovingROINodelet::imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg, const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg) {

	// Update the camera model
	model_.fromCameraInfo(l_info_msg, r_info_msg);

	left_cam_info_ = *l_info_msg;
	right_cam_info_ = *r_info_msg;

	left_cam_info_.roi.x_offset = 80;
	left_cam_info_.roi.y_offset = 60;
	left_cam_info_.roi.width = left_cam_info_.width - 80*2;/// 2;
	left_cam_info_.roi.height = left_cam_info_.height - 60*2; /// 2;

	right_cam_info_.roi.x_offset = 80;
	right_cam_info_.roi.y_offset = 60;
	right_cam_info_.roi.width = right_cam_info_.width -80*2;/// 2;
	right_cam_info_.roi.height = right_cam_info_.height -60*2;/// 2;

	left_image_pub_.publish(l_image_msg);
	right_image_pub_.publish(r_image_msg);
	left_camera_info_pub_.publish(left_cam_info_);
	right_camera_info_pub_.publish(right_cam_info_);
}

}
// Register nodelet
PLUGINLIB_EXPORT_CLASS(lrm_stereo::MovingROINodelet, nodelet::Nodelet)
