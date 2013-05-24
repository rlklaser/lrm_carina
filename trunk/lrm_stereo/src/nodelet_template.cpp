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
 * @file nodelet_template.cpp
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

#include <image_geometry/stereo_camera_model.h>
//#include <opencv2/calib3d/calib3d.hpp>

//#include <sensor_msgs/image_encodings.h>
//#include <stereo_msgs/DisparityImage.h>

//#include <lrm_stereo/DisparitySGBMConfig.h>
//#include <dynamic_reconfigure/server.h>

namespace lrm_stereo {

using namespace sensor_msgs;

using namespace message_filters::sync_policies;

class FeatureMatcherNodelet: public nodelet::Nodelet {
	boost::shared_ptr<image_transport::ImageTransport> it_;

	// Subscriptions
	image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
	message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
	typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
	typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
	typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
	typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
	boost::shared_ptr<ExactSync> exact_sync_;
	boost::shared_ptr<ApproximateSync> approximate_sync_;
	// Publications
	boost::mutex connect_mutex_;

	ros::Publisher pub_matches_;

	// Dynamic reconfigure
//	boost::recursive_mutex config_mutex_;
//	typedef lrm_stereo::DisparitySGBMConfig Config;
//	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
//	boost::shared_ptr<ReconfigureServer> reconfigure_server_;

	// Processing state (note: only safe because we're single-threaded!)
	image_geometry::StereoCameraModel model_;

	virtual void onInit();

	void connectCb();

	void imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg, const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg);

};


void FeatureMatcherNodelet::onInit() {
	ros::NodeHandle &nh = getNodeHandle();
	ros::NodeHandle &private_nh = getPrivateNodeHandle();

	it_.reset(new image_transport::ImageTransport(nh));

	// Synchronize inputs. Topic subscriptions happen on demand in the connection
	// callback. Optionally do approximate synchronization.
	int queue_size;
	private_nh.param("queue_size", queue_size, 5);
	bool approx;
	private_nh.param("approximate_sync", approx, false);
	if (approx) {
		approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size), sub_l_image_, sub_l_info_, sub_r_image_, sub_r_info_));
		approximate_sync_->registerCallback(boost::bind(&FeatureMatcherNodelet::imageCb, this, _1, _2, _3, _4));
	} else {
		exact_sync_.reset(new ExactSync(ExactPolicy(queue_size), sub_l_image_, sub_l_info_, sub_r_image_, sub_r_info_));
		exact_sync_->registerCallback(boost::bind(&FeatureMatcherNodelet::imageCb, this, _1, _2, _3, _4));
	}

//	// Set up dynamic reconfiguration
//	ReconfigureServer::CallbackType f = boost::bind(&DisparitySGBMNodelet::configCb, this, _1, _2);
//	reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
//	reconfigure_server_->setCallback(f);

	// Monitor whether anyone is subscribed to the output
	ros::SubscriberStatusCallback connect_cb = boost::bind(&FeatureMatcherNodelet::connectCb, this);
	// Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
	boost::lock_guard < boost::mutex > lock(connect_mutex_);
	//pub_disparity_ = nh.advertise<DisparityImage>("surf_cloud", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void FeatureMatcherNodelet::connectCb() {
	boost::lock_guard < boost::mutex > lock(connect_mutex_);
	if (pub_matches_.getNumSubscribers() == 0) {
		sub_l_image_.unsubscribe();
		sub_l_info_.unsubscribe();
		sub_r_image_.unsubscribe();
		sub_r_info_.unsubscribe();
	} else if (!sub_l_image_.getSubscriber()) {
		ros::NodeHandle &nh = getNodeHandle();
		// Queue size 1 should be OK; the one that matters is the synchronizer queue size.
		/// @todo Allow remapping left, right?
		image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
		sub_l_image_.subscribe(*it_, "left/image_rect", 1, hints);
		sub_l_info_.subscribe(nh, "left/camera_info", 1);
		sub_r_image_.subscribe(*it_, "right/image_rect", 1, hints);
		sub_r_info_.subscribe(nh, "right/camera_info", 1);
	}
}

void FeatureMatcherNodelet::imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg, const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg) {

	// Update the camera model
	model_.fromCameraInfo(l_info_msg, r_info_msg);
}

}
// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lrm_stereo::FeatureMatcherNodelet, nodelet::Nodelet)
