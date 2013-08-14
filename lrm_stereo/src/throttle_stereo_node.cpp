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
 * @file throttle_stereo.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jul 18, 2013
 *
 */

/*
 *
 * code copied from : https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/grasp_playpen/src/throttle_stereo.cpp
 *
 */

/*
 Node to output throttled versions of the stereo camera raw images and camera_info (which need to stay synchronized)
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>

#include "lrm_stereo/ThrottleStereoConfig.h";

using std::string;
using namespace sensor_msgs;
using namespace message_filters;

//typedef sync_policies::ExactTime<Image, Image> StereoCameraSyncPolicy;
typedef sync_policies::ApproximateTime<Image, Image> StereoCameraSyncPolicy;

// class for broadcasting synchronized, throttled versions of a stereo camera topic
class StereoCameraThrottler {
private:
	ros::NodeHandle* nh_;
	ros::NodeHandle* nh_priv_;
	ros::Publisher left_image_pub_;
	ros::Publisher right_image_pub_;
	ros::Publisher left_camera_info_pub_;
	ros::Publisher right_camera_info_pub_;
	message_filters::Subscriber<Image> left_sub_;
	message_filters::Subscriber<Image> right_sub_;
	Synchronizer<StereoCameraSyncPolicy> sync_;
	ros::Subscriber left_info_sub_;
	ros::Subscriber right_info_sub_;
	CameraInfo left_cam_info_;
	CameraInfo right_cam_info_;
	bool received_left_cam_info_;
	bool received_right_cam_info_;
	double msg_rate_;
	double last_msg_time_;
	string image_topic_;
	boost::recursive_mutex config_mutex_;
	boost::shared_ptr<ReconfigureServer> reconfigure_server_;

public:

	StereoCameraThrottler(ros::NodeHandle* nh, ros::NodeHandle* nh_priv) {
		//store the topic and initial rate and initialize the last message time
		image_topic_ = image_topic;
		msg_rate_ = rate;
		last_msg_time_ = 0;
		received_left_cam_info_ = 0;
		received_right_cam_info_ = 0;

		int queue_size;
		bool approx;

		nh_priv_->param("queue_size", queue_size, 5);
		nh_priv_->param("approximate_sync", approx, false);
		nh_priv_->param<string>("input_namespace", image_topic_, "stereo_camera");
		nh_priv_->param<string>("output_namespace", out_topic, "stereo");
		nh_priv_->param<double>("rate", rate, 10);

		left_sub_(node_, image_topic + string("/left/image_raw"), 1),
		right_sub_(node_, image_topic + string("/right/image_raw"), 1),

		sync_(StereoCameraSyncPolicy(50), left_sub_, right_sub_)

		//subscribe to the left and right camera info messages
		left_info_sub_ = node_.subscribe(image_topic_ + string("/left/camera_info"), 5, &StereoCameraThrottler::left_info_cb, this);
		right_info_sub_ = node_.subscribe(image_topic_ + string("/right/camera_info"), 5, &StereoCameraThrottler::right_info_cb, this);


		//register the callback for the topic synchronizer
		sync_.registerCallback(boost::bind(&StereoCameraThrottler::stereo_cb, this, _1, _2));

		//broadcast the throttled versions on out_topic
		left_image_pub_ = node_.advertise<Image>(out_topic + string("/left/image_raw"), 10);
		right_image_pub_ = node_.advertise<Image>(out_topic + string("/right/image_raw"), 10);
		left_camera_info_pub_ = node_.advertise<CameraInfo>(out_topic + string("/left/camera_info"), 10);
		right_camera_info_pub_ = node_.advertise<CameraInfo>(out_topic + string("/right/camera_info"), 10);

		ROS_INFO("done init for %s", (image_topic_ + string("/left/image_raw")).c_str());
	}

	//record the left camera info (once)
	void left_info_cb(const CameraInfoConstPtr& cam_info) {
		if (received_left_cam_info_)
			return;
		left_cam_info_ = *cam_info;
		received_left_cam_info_ = 1;
		left_camera_info_pub_.publish(left_cam_info_);
		ROS_INFO("got left_cam_info for %s", image_topic_.c_str());
	}

	//record the right camera info (once)
	void right_info_cb(const CameraInfoConstPtr& cam_info) {
		if (received_right_cam_info_)
			return;
		right_cam_info_ = *cam_info;
		received_right_cam_info_ = 1;
		right_camera_info_pub_.publish(right_cam_info_);
		ROS_INFO("got right_cam_info for %s", image_topic_.c_str());
	}

	//re-broadcast the stereo images and their info, throttled to the desired frequencies
	void stereo_cb(const ImageConstPtr& left_image, const ImageConstPtr& right_image) {
		//ROS_INFO("got synchronized image for %s", image_topic_.c_str());
		if (ros::Time::now().toSec() - last_msg_time_ > 1 / msg_rate_) {
			left_image_pub_.publish(left_image);
			right_image_pub_.publish(right_image);
			if (received_left_cam_info_) {
				left_cam_info_.header.stamp = left_image->header.stamp;
				left_camera_info_pub_.publish(left_cam_info_);
			}
			if (received_right_cam_info_) {
				right_cam_info_.header.stamp = right_image->header.stamp;
				right_camera_info_pub_.publish(right_cam_info_);
			}
			last_msg_time_ = ros::Time::now().toSec();
			//ROS_INFO("broadcasting synchronized image for %s", image_topic_.c_str());
		}
	}

	void setRate(double rate) {
		msg_rate_ = rate;
	}

	void reconfig(lrm_stereo::ThrottleStereoConfig &config, uint32_t level) {
		setRate(config.rate);
		ROS_WARN_STREAM("throttle stereo reconfigured, rate=" << config.rate);
	}
};


StereoCameraThrottler *throttler;



int main(int argc, char** argv) {
	//if (argc < 4) {
	//	puts("usage: throttle_stereo NARROW_STEREO_RATE NARROW_STEREO_TEXTURED_RATE WIDE_STEREO_RATE\n\n");
	//	return 1;
	//}

	//double narrow_stereo_rate = atof(argv[1]);
	//double narrow_stereo_textured_rate = atof(argv[2]);
	//double wide_stereo_rate = atof(argv[3]);

	ros::init(argc, argv, "throttle_stereo_images");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	string in_ns;
	string out_ns;
	double rate;

	nh_priv.param<string>("input_namespace", in_ns, "stereo_camera");
	nh_priv.param<string>("output_namespace", out_ns, "stereo");
	nh_priv.param<double>("rate", rate, 10);

	//create throttler-synchronizers for the wide, narrow, and narrow_textured stereo cameras
	//StereoCameraThrottler *narrow_stereo_throttler = new StereoCameraThrottler(string("/narrow_stereo"), string("/narrow_stereo_throttled"), narrow_stereo_rate);
	//StereoCameraThrottler *narrow_stereo_textured_throttler = new StereoCameraThrottler(string("/narrow_stereo_textured"), string("/narrow_stereo_textured_throttled"), narrow_stereo_textured_rate);
	//StereoCameraThrottler *wide_stereo_throttler = new StereoCameraThrottler(string("/wide_stereo"), string("/wide_stereo_throttled"), wide_stereo_rate);

	throttler = new StereoCameraThrottler(in_ns, out_ns, rate);

	dynamic_reconfigure::Server<lrm_stereo::ThrottleStereoConfig> srv;
	dynamic_reconfigure::Server<lrm_stereo::ThrottleStereoConfig>::CallbackType f = boost::bind(&reconfig, _1, _2);
	srv.setCallback(f);

	ros::spin();

	//delete narrow_stereo_throttler;
	//delete narrow_stereo_textured_throttler;
	//delete wide_stereo_throttler;

	delete throttler;

	return 0;
}
