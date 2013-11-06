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
 * @file throttle_stereo.h
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 5, 2013
 *
 */

#ifndef THROTTLE_STEREO_H_
#define THROTTLE_STEREO_H_

/*
 *
 * code based on: https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/grasp_playpen/src/throttle_stereo.cpp
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

#include "lrm_stereo/ThrottleStereoConfig.h"

using namespace sensor_msgs;
using namespace message_filters;

namespace lrm_stereo {

//typedef sync_policies::ExactTime<Image, Image> StereoCameraSyncPolicy;
typedef sync_policies::ApproximateTime<Image, Image> StereoCameraSyncPolicy;
typedef message_filters::Subscriber<Image> ImageSubscriber;
typedef Synchronizer<StereoCameraSyncPolicy> ImageSynchronizer;

// class for broadcasting synchronized, throttled versions of a stereo camera topic
class StereoCameraThrottler {
private:
	ros::NodeHandle& nh_;
	ros::NodeHandle& nh_priv_;
	ros::Publisher left_image_pub_;
	ros::Publisher right_image_pub_;
	ros::Publisher left_camera_info_pub_;
	ros::Publisher right_camera_info_pub_;
	boost::shared_ptr<ImageSubscriber> left_sub_;
	boost::shared_ptr<ImageSubscriber> right_sub_;
	boost::shared_ptr<ImageSynchronizer> sync_;
	ros::Subscriber left_info_sub_;
	ros::Subscriber right_info_sub_;
	CameraInfo left_cam_info_;
	CameraInfo right_cam_info_;
	bool received_left_cam_info_;
	bool received_right_cam_info_;
	double msg_rate_;
	ros::Time last_msg_time_;
	std::string image_topic_;
	//boost::recursive_mutex config_mutex_;
	dynamic_reconfigure::Server<ThrottleStereoConfig> srv_;
public:
	StereoCameraThrottler(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
	//virtual ~StereoCameraThrottler();
	void reconfig(ThrottleStereoConfig &config, uint32_t level);
	void setRate(double rate);
protected:
	void stereo_cb(const ImageConstPtr& left_image, const ImageConstPtr& right_image);
	void right_info_cb(const CameraInfoConstPtr& cam_info);
	void left_info_cb(const CameraInfoConstPtr& cam_info);
};
}

#endif /* THROTTLE_STEREO_H_ */
