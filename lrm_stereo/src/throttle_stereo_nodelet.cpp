#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>

#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <dynamic_reconfigure/server.h>

#include "lrm_stereo/ThrottleStereoConfig.h"

namespace lrm_stereo {

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class ThrottleStereoNodelet: public nodelet::Nodelet {
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
	ros::Publisher left_image_pub_;
	ros::Publisher right_image_pub_;
	ros::Publisher left_camera_info_pub_;
	ros::Publisher right_camera_info_pub_;

	// Dynamic reconfigure
	boost::recursive_mutex config_mutex_;
	typedef lrm_stereo::ThrottleStereoConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
	boost::shared_ptr<ReconfigureServer> reconfigure_server_;

	double last_msg_time_;
	double msg_rate_;
	int queue_size_;
	std::string in_ns_;
	std::string out_ns_;

	virtual void onInit();
	void connectCb();
	void imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg, const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg);
	void configCb(Config &config, uint32_t level);

};

void ThrottleStereoNodelet::onInit() {
	ros::NodeHandle &nh = getNodeHandle();
	ros::NodeHandle &private_nh = getPrivateNodeHandle();

	it_.reset(new image_transport::ImageTransport(nh));

	last_msg_time_ = 0;

	// Synchronize inputs. Topic subscriptions happen on demand in the connection
	// callback. Optionally do approximate synchronization.

	private_nh.param("queue_size", queue_size_, 5);
	bool approx;
	private_nh.param("approximate_sync", approx, false);
	if (approx) {
		approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_), sub_l_image_, sub_l_info_, sub_r_image_, sub_r_info_));
		approximate_sync_->registerCallback(boost::bind(&ThrottleStereoNodelet::imageCb, this, _1, _2, _3, _4));
	} else {
		exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_), sub_l_image_, sub_l_info_, sub_r_image_, sub_r_info_));
		exact_sync_->registerCallback(boost::bind(&ThrottleStereoNodelet::imageCb, this, _1, _2, _3, _4));
	}

	private_nh.param<std::string>("input_namespace", in_ns_, "stereo_camera");
	private_nh.param<std::string>("output_namespace", out_ns_, "stereo");
	private_nh.param<double>("rate", msg_rate_, 10);

	// Set up dynamic reconfiguration
	ReconfigureServer::CallbackType f = boost::bind(&ThrottleStereoNodelet::configCb, this, _1, _2);
	reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
	reconfigure_server_->setCallback(f);

	// Monitor whether anyone is subscribed to the output
	ros::SubscriberStatusCallback connect_cb = boost::bind(&ThrottleStereoNodelet::connectCb, this);
	// Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
	boost::lock_guard < boost::mutex > lock(connect_mutex_);

	//broadcast the throttled versions on out_topic
	left_image_pub_ = nh.advertise<Image>(out_ns_ + "/left/image_raw", 1, connect_cb, connect_cb);
	right_image_pub_ = nh.advertise<Image>(out_ns_ + "/right/image_raw", 1, connect_cb, connect_cb);
	left_camera_info_pub_ = nh.advertise<CameraInfo>(out_ns_ + "/left/camera_info", 1, connect_cb, connect_cb);
	right_camera_info_pub_ = nh.advertise<CameraInfo>(out_ns_ + "/right/camera_info", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void ThrottleStereoNodelet::connectCb() {
	boost::lock_guard < boost::mutex > lock(connect_mutex_);
	if (left_image_pub_.getNumSubscribers() == 0) {
		sub_l_image_.unsubscribe();
		sub_l_info_.unsubscribe();
		sub_r_image_.unsubscribe();
		sub_r_info_.unsubscribe();
	} else if (!sub_l_image_.getSubscriber()) {
		ros::NodeHandle &nh = getNodeHandle();
		// Queue size 1 should be OK; the one that matters is the synchronizer queue size.
		/// @todo Allow remapping left, right?
		image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
		sub_l_image_.subscribe(*it_, in_ns_ + "/left/image_rect", 1, hints);
		sub_l_info_.subscribe(nh, in_ns_ + "/left/camera_info", 1);
		sub_r_image_.subscribe(*it_, in_ns_ + "/right/image_rect", 1, hints);
		sub_r_info_.subscribe(nh, in_ns_ + "/right/camera_info", 1);
	}
}

void ThrottleStereoNodelet::imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg, const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg) {
	//ROS_INFO("got synchronized image for %s", image_topic_.c_str());
	if (ros::Time::now().toSec() - last_msg_time_ > 1 / msg_rate_) {

		left_image_pub_.publish(l_image_msg);
		right_image_pub_.publish(r_image_msg);
		left_camera_info_pub_.publish(l_info_msg);
		right_camera_info_pub_.publish(r_info_msg);

		last_msg_time_ = ros::Time::now().toSec();
		//ROS_INFO("broadcasting synchronized image for %s", image_topic_.c_str());
	}
}

void ThrottleStereoNodelet::configCb(Config &config, uint32_t level) {

}

} // namespace stereo_image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lrm_stereo::ThrottleStereoNodelet, nodelet::Nodelet)
