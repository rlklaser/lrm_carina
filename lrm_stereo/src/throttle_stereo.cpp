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

#include "lrm_stereo/throttle_stereo.h"

namespace lrm_stereo
{

StereoCameraThrottler::StereoCameraThrottler(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
	: nh_(nh), nh_priv_(nh_priv)
{
	//store the topic and initial rate and initialize the last message time
	last_msg_time_ = ros::Time(0);
	received_left_cam_info_ = 0;
	received_right_cam_info_ = 0;

	std::string out_topic;

	nh_priv_.param<std::string>("input_namespace", image_topic_, "stereo_camera");
	nh_priv_.param<std::string>("output_namespace", out_topic, "stereo");
	nh_priv_.param<double>("rate", msg_rate_, 10);

	nh_priv_.param<bool>("approximate_sync", approximate_sync_, false);

	//subscribe to the left and right camera info messages
	left_info_sub_ = nh_.subscribe(image_topic_ + std::string("/left/camera_info"), 1, &StereoCameraThrottler::left_info_cb, this);
	right_info_sub_ = nh_.subscribe(image_topic_ + std::string("/right/camera_info"), 1, &StereoCameraThrottler::right_info_cb, this);

	left_sub_.reset(new ImageSubscriber(nh_, image_topic_ + std::string("/left/image_raw"), 1));
	right_sub_.reset(new ImageSubscriber(nh_, image_topic_ + std::string("/right/image_raw"), 1));

	//register the callback for the topic synchronizer
	if(approximate_sync_) {
		sync_ap_.reset(new ImageSynchronizerAp(StereoCameraApSyncPolicy(5), *left_sub_, *right_sub_));
		sync_ap_->registerCallback(boost::bind(&StereoCameraThrottler::stereo_cb, this, _1, _2));
	}
	else {
		sync_ex_.reset(new ImageSynchronizerEx(StereoCameraExSyncPolicy(5), *left_sub_, *right_sub_));
		sync_ex_->registerCallback(boost::bind(&StereoCameraThrottler::stereo_cb, this, _1, _2));
	}

	dynamic_reconfigure::Server<lrm_stereo::ThrottleStereoConfig>::CallbackType f = boost::bind(&StereoCameraThrottler::reconfig, this, _1, _2);
	srv_.setCallback(f);

	//broadcast the throttled versions on out_topic
	left_image_pub_ = nh_.advertise<Image>(out_topic + std::string("/left/image_raw"), 2);
	right_image_pub_ = nh_.advertise<Image>(out_topic + std::string("/right/image_raw"), 2);
	left_camera_info_pub_ = nh_.advertise<CameraInfo>(out_topic + std::string("/left/camera_info"), 2);
	right_camera_info_pub_ = nh_.advertise<CameraInfo>(out_topic + std::string("/right/camera_info"), 2);

	ROS_INFO_STREAM("done init for " << image_topic_ << "/left/image_raw");
}

//record the left camera info (once)
void StereoCameraThrottler::left_info_cb(const CameraInfoConstPtr& cam_info)
{
	if (received_left_cam_info_)
		return;
	left_cam_info_ = *cam_info;
	received_left_cam_info_ = 1;
	left_camera_info_pub_.publish(left_cam_info_);
	ROS_INFO("got left_cam_info for %s", image_topic_.c_str());
}

//record the right camera info (once)
void StereoCameraThrottler::right_info_cb(const CameraInfoConstPtr& cam_info)
{
	if (received_right_cam_info_)
		return;
	right_cam_info_ = *cam_info;
	received_right_cam_info_ = 1;
	right_camera_info_pub_.publish(right_cam_info_);
	ROS_INFO("got right_cam_info for %s", image_topic_.c_str());
}

//re-broadcast the stereo images and their info, throttled to the desired frequencies
void StereoCameraThrottler::stereo_cb(const ImageConstPtr& left_image, const ImageConstPtr& right_image)
{
	//ROS_INFO("got synchronized image for %s", image_topic_.c_str());
	if (ros::Time::now().toSec() - last_msg_time_.toSec() > 1 / msg_rate_)
	{
		last_msg_time_ = ros::Time::now();

		ros::Time stamp = left_image->header.stamp;

		left_image_pub_.publish(left_image);

		if(approximate_sync_) {
			//make all image same stamp
			Image right_image_copy = *right_image;
			right_image_copy.header.stamp = stamp;
			right_image_pub_.publish(right_image_copy);
		}
		else {
			right_image_pub_.publish(right_image);
		}

		if (received_left_cam_info_)
		{
			left_cam_info_.header.stamp = stamp;
			left_camera_info_pub_.publish(left_cam_info_);
		}
		if (received_right_cam_info_)
		{
			right_cam_info_.header.stamp = stamp;
			right_camera_info_pub_.publish(right_cam_info_);
		}

		//ROS_INFO("broadcasting synchronized image for %s", image_topic_.c_str());
	}
}

void StereoCameraThrottler::setRate(double rate)
{
	msg_rate_ = rate;
}

void StereoCameraThrottler::reconfig(lrm_stereo::ThrottleStereoConfig &config, uint32_t level)
{
	setRate(config.rate);
	ROS_WARN_STREAM("throttle stereo reconfigured, rate=" << config.rate);
}

}

