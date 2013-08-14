/*
 * bumblebee2.cpp
 *
 *    Created on: May 7, 2010
 *        Author: Soonhac Hong (sh2723@columbia.edu)
 *         Note : Most of code of this file is borrowed from camera1394.cpp of camera1394 package.
 * Modification : In video mode, 640x480_stereo_mono is added to assign the video mode to DC1394_VIDEO_MODE_640x480_MONO16.
 *                As mono 16bit data(MSB-right camera, LSB-left camera) are captured from bumblebee2,
 *                deinterlaced image is divided into right and left image and published as sensor_msgs/image in read().
 *        Usage : $roslaunch bumblebee2 Bumblebee2.launch
 *                $rosrun image_view image_view image:=/bumblebee2/left/image_raw
 *                $rosrun image_view image_view image:=/bumblebee2/right/image_raw
 *                To view stereo image and disparity image,
 *                $ROS_NAMESPACE=bumblebee2 rosrun stereo_image_proc stereo_image_proc
 *                $rosrun image_view stereo_view stereo:=/bumblebee2 image:=image_mono
 */

// $Id: camera1394.cpp 28690 2010-04-09 15:36:33Z joq $
///////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2009, 2010 Patrick Beeson, Jack O'Quin
//  ROS port of the Player 1394 camera driver.
//
// Copyright (C) 2004 Nate Koenig, Andrew Howard
//  Player driver for IEEE 1394 digital camera capture
//
// Copyright (C) 2000-2003 Damien Douxchamps, Dan Dennedy
//  Bayer filtering from libdc1394
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//
///////////////////////////////////////////////////////////////////////////
#include <signal.h>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/SensorLevels.h>
#include "bumblebee2.h"
#include "bumblebee2/Bumblebee2Config.h"

namespace enc = sensor_msgs::image_encodings;

using namespace camera_info_manager;

/** @file

 @brief camera1394 is a ROS driver for 1394 Firewire digital cameras.

 This is a ROS port of the Player driver for 1394 cameras, using
 libdc1394.  It provides a reliable driver with minimal dependencies,
 intended to fill a role in the ROS image pipeline similar to the other
 ROS camera drivers.

 The ROS image pipeline provides Bayer filtering at a higher level (in
 image_proc).  In some cases it is useful to run the driver without the
 entire image pipeline, so libdc1394 Bayer decoding is also provided.

 @par Advertises

 - \b camera/image_raw topic (sensor_msgs/Image) raw 2D camera images
 (only raw if \b bayer_method is \b NONE).

 - \b camera/camera_info topic (sensor_msgs/CameraInfo) Calibration
 information for each image.


 @par Subscribes

 - None


 @par Parameters

 - \b frame_id : @b [string] camera frame of reference
 (Default: device node name)
 - \b guid : @b [string] The guid of the camera (Default: "NONE")
 - \b fps : @b [real] Frames per second (Default: 15.0)
 - \b iso_speed : @b [int] ISO transfer speed (Default: 400)
 - \b video_mode : @b [string] Desired image resolution and type (Default "800x600_mono").  The driver supports the following values:
 "320x240_yuv422"
 "640x480_mono"
 "640x480_yuv422"
 "640x480_rgb"
 "800x600_mono"
 "800x600_yuv422"
 "1024x768_mono"
 "1024x768_yuv422"
 "1280x960_mono"
 "1280x960_yuv422"
 - \b bayer_pattern : @b [string] The pattern of the Bayer filter to use (Default: "NONE").  The driver supports the following values:
 "BGGR"
 "GRBG"
 "RGGB"
 "GBRG"
 "NONE"
 - \b bayer_method : @b [string] The type of Bayer processing to perform (Default: "NONE").  The driver supports the following values:
 "NONE"
 "DownSample" (1/2 size image)
 "Nearest"
 "Bilinear"
 "HQ"
 "VNG"
 "AHD"
 - \b exposure : @b [int] Sets the camera exposure feature to value.
 - \b shutter : @b [int] Sets the camera shutter feature to value.  -1 turns on auto shutter.
 - \b whitebalance : @b [string] (e.g. "2000 2000") Sets the Blue/U and Red/V components of white balance.  "auto" turns on auto white balance.
 - \b gain : @b [int] Sets the camera gain feature to value.  -1 turns on auto gain.
 - \b brightness : @b [int] Sets the camera brightness feature to value.


 @todo Make array of supported image encoding values, check parameter
 settings against that. Make enum type for dynamic reconfiguration.

 */

void sigsegv_handler(int sig);

class Bumblebee2Node {
private:
	ros::NodeHandle privNH_; // private node handle
	image_transport::ImageTransport *it_;
	std::string camera_name_;
	std::string frame_id_;
	sensor_msgs::Image image_;
	sensor_msgs::Image left_image_;
	sensor_msgs::Image right_image_;
	sensor_msgs::CameraInfo left_cam_info_;
	sensor_msgs::CameraInfo right_cam_info_;

	CameraInfoManager *cinfo_;

	// initial parameters
	std::string bayerMethod_;
	std::string bayerPattern_;
	std::string encoding_;
	double frameRate_;
	std::string guid_;
	int isoSpeed_;
	std::string videoMode_;

	// reconfigurable parameters
	int brightness_;
	int exposure_;
	int gain_;
	int shutter_;
	std::string whitebalance_;

	/** 1394 camera device driver */
	bumblebee2::Camera1394 *dev_;

	/** dynamic parameter config message */
	bumblebee2::Bumblebee2Config config_;

	/** image transport publish interface */
	image_transport::CameraPublisher left_image_pub_;
	image_transport::CameraPublisher right_image_pub_;

public:

	Bumblebee2Node() :
			dev_(0) {
		privNH_ = ros::NodeHandle("~");
		getInitParams();

		// set reconfigurable parameter defaults (all to automatic)
		brightness_ = -1;
		encoding_ = "";
		exposure_ = -1;
		gain_ = -1;
		shutter_ = -1;
		whitebalance_ = "auto";

		cinfo_ = new CameraInfoManager(privNH_, camera_name_);

		it_ = new image_transport::ImageTransport(privNH_);
		left_image_pub_ = it_->advertiseCamera("left/image_raw", 1);
		right_image_pub_ = it_->advertiseCamera("right/image_raw", 2);
	}

	~Bumblebee2Node() {
		delete cinfo_;
		delete it_;
	}

	/** Close camera device */
	void closeCamera() {
		if (dev_) {
			dev_->close();
			delete dev_;
			dev_ = NULL;
		}
	}

	/** get initial parameters (only when node starts). */
	void getInitParams(void) {
		if (!privNH_.getParam("frame_id", frame_id_)) {
			frame_id_ = "camera";
		}

		// resolve frame ID using tf_prefix parameter
		std::string tf_prefix = tf::getPrefixParam(privNH_);
		ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
		frame_id_ = tf::resolve(tf_prefix, frame_id_);

		// default camera name is base name of frame ID
		size_t basepos = frame_id_.find_last_of("/");
		camera_name_ = frame_id_.substr(basepos + 1);

		// read other parameters
		privNH_.param("bayer_method", bayerMethod_, std::string("NONE"));
		privNH_.param("bayer_pattern", bayerPattern_, std::string("NONE"));
		privNH_.param("fps", frameRate_, 15.0);
		privNH_.param("guid", guid_, std::string("NONE"));
		privNH_.param("iso_speed", isoSpeed_, 400);
		privNH_.param("video_mode", videoMode_, std::string("800x600_mono"));

		ROS_INFO_STREAM("[" << camera_name_ << "] video mode: " << videoMode_ << ", frame ID: " << frame_id_);
	}

	/** Update reconfigurable parameter.
	 *
	 * This is done every frame, so we use getParamCached() to avoid
	 * unnecessary parameter server requests.
	 *
	 * @param name ROS parameter private name
	 * @param val current parameter value; updated on exit
	 * @return true if @a val changed this time
	 */
	bool inline updateParam(const std::string &name, int &val) {
		bool changed = false;
		int prev = val;
		if (privNH_.getParamCached(name, val) && (val != prev))
			changed = true;
		return changed;
	}

	/** Open the camera device.
	 *
	 * @return true, if successful
	 */
	bool openCamera() {
		bool success = true;
		try {
			dev_ = new bumblebee2::Camera1394();
			if (dev_->open(guid_.c_str(), videoMode_.c_str(), frameRate_, isoSpeed_, bayerPattern_.c_str(), bayerMethod_.c_str()) == 0) {
				ROS_INFO_STREAM("[" << camera_name_ << "] connected to device, ID: " << dev_->device_id_);
			}

		} catch (bumblebee2::Exception& e) {
			ROS_FATAL_STREAM("[" << camera_name_ << "] exception opening device: " << e.what());
			success = false;
		}

		return success;
	}

	/** Update reconfigurable parameter (for strings). */
	bool inline updateParam(const std::string &name, std::string &val) {
		bool changed = false;
		std::string prev = val;
		if (privNH_.getParamCached(name, val) && (val != prev))
			changed = true;
		return changed;
	}

	/** Check for changes in reconfigurable parameters. */
	void getParameters() {
		static bool first_cycle = true;

		if (updateParam("brightness", brightness_) || first_cycle) {
			if (dev_->setBrightness(brightness_) >= 0) {
				if (brightness_ >= 0)
					ROS_INFO_STREAM("[" << camera_name_ << "] Brightness set to " << brightness_);
				else
					ROS_INFO_STREAM("[" << camera_name_ << "] Auto Brightness set");
			}
		}

		if (updateParam("exposure", exposure_) || first_cycle) {
			if (dev_->setExposure(exposure_) >= 0) {
				if (exposure_ >= 0)
					ROS_INFO_STREAM("[" << camera_name_ << "] Exposure set to " << exposure_);
				else
					ROS_INFO_STREAM("[" << camera_name_ << "] Auto Exposure set");
			}
		}

		if (updateParam("gain", gain_) || first_cycle) {
			if (dev_->setGain(gain_) >= 0) {
				if (gain_ >= 0)
					ROS_INFO_STREAM("[" << camera_name_ << "] Gain set to " << gain_);
				else
					ROS_INFO_STREAM("[" << camera_name_ << "] Auto Gain set");
			}
		}

		if (updateParam("shutter", shutter_) || first_cycle) {
			if (dev_->setShutter(shutter_) >= 0) {
				if (shutter_ >= 0)
					ROS_INFO_STREAM("[" << camera_name_ << "] Shutter set to " << shutter_);
				else
					ROS_INFO_STREAM("[" << camera_name_ << "] Auto Shutter set");
			}
		}

		if (updateParam("whitebalance", whitebalance_) || first_cycle) {
			if (dev_->setWhiteBalance(whitebalance_.c_str()) >= 0) {
				if (whitebalance_ != "auto")
					ROS_INFO_STREAM("[" << camera_name_ << "] Whitebalance set to " << whitebalance_);
				else
					ROS_INFO_STREAM("[" << camera_name_ << "] Auto Whitebalance set");
			}
		}

		if (updateParam("encoding", encoding_) || first_cycle) {
			ROS_INFO_STREAM("[" << camera_name_ << "] Encoding set to " << encoding_);
		}

		first_cycle = false;
	}

	/** Read camera data */
	void read() {
		// get current CameraInfo data
		left_cam_info_ = right_cam_info_ = cinfo_->getCameraInfo();
		left_image_.header.frame_id = right_image_.header.frame_id = image_.header.frame_id = left_cam_info_.header.frame_id = right_cam_info_.header.frame_id = frame_id_;

		//update bumblebee2 calibration data
		//updateBumblebee2CalibrationData();

		try {
			// Read data from the Camera
			dev_->readData(image_);

			left_cam_info_.header.stamp = right_cam_info_.header.stamp = left_image_.header.stamp = right_image_.header.stamp = image_.header.stamp;
			left_cam_info_.height = right_cam_info_.height = left_image_.height = right_image_.height = image_.height;
			left_cam_info_.width = right_cam_info_.width = left_image_.width = right_image_.width = image_.width;
			if (encoding_ != "")
				image_.encoding = encoding_; // override driver setting
			left_image_.encoding = right_image_.encoding = image_.encoding;

			//Split image into left image and right image
			left_image_.step = right_image_.step = image_.step;
			int image_size = image_.height * image_.step;
			left_image_.data.resize(image_size);
			right_image_.data.resize(image_size);
			memcpy(&right_image_.data[0], &image_.data[0], image_size); // the image of right camera is the first half of the deinterlaced image.
			memcpy(&left_image_.data[0], &image_.data[image_size], image_size); // the image of left camera is the second half of the deinterlaced image.

			// Publish it via image_transport
			left_image_pub_.publish(left_image_, left_cam_info_);
			right_image_pub_.publish(right_image_, right_cam_info_);
		} catch (bumblebee2::Exception& e) {
			ROS_WARN_STREAM("[" << camera_name_ << "] Exception reading data: " << e.what());
			//TODO: shut down and exit?
		}
	}

	/** Update the bumblebee2 calibration data */
	/** Author : Soonhac Hong (sh2723@columbia.edu) */
	/** Date : 5/24/2010 */
	/** Note : Calibration data is needed to show disparity image using image_view with stereo_view.*/
	void updateBumblebee2CalibrationData() {
		double left_D_data[] = { -0.29496962080028677, 0.12120859315219049, -0.0019941265153862824, 0.0012058185627261283, 0.0 };
		double left_K_data[] = { 543.60636929659358, 0.0, 321.7411723319629, 0.0, 543.25622524820562, 268.04452669345528, 0.0, 0.0, 1.0 };
		double left_R_data[] = { 0.99980275533925467, -0.018533834763323875, -0.0071377436911170388, 0.018542709766871161, 0.99982737377597841, 0.0011792212393866724, 0.0071146560377753926, -0.0013113417539480422, 0.9999738306837177 };
		double left_P_data[] = { 514.20203529502226, 0.0, 334.37528610229492, 0.0, 0.0, 514.20203529502226, 268.46113204956055, 0.0, 0.0, 0.0, 1.0, 0.0 };
		double right_D_data[] = { -0.2893208200535437, 0.11215776927066376, -0.0003854904042866552, 0.00081197271575971614, 0.0 };
		double right_K_data[] = { 541.66040340873735, 0.0, 331.73470962829737, 0.0, 541.60313005445187, 265.72960150703699, 0.0, 0.0, 1.0 };
		double right_R_data[] = { 0.99986888001551244, -0.012830354497672055, -0.0098795131453283894, 0.012818040762902759, 0.99991698911455085, -0.001308705884349387, 0.0098954841986237611, 0.001181898284639702, 0.99995034002140304 };
		double right_P_data[] = { 514.20203529502226, 0.0, 334.37528610229492, -232.44101555000066, 0.0, 514.20203529502226, 268.46113204956055, 0.0, 0.0, 0.0, 1.0, 0.0 };

		memcpy(&left_cam_info_.D[0], &left_D_data[0], sizeof(left_D_data));
		memcpy(&left_cam_info_.K[0], &left_K_data[0], sizeof(left_K_data));
		memcpy(&left_cam_info_.R[0], &left_R_data[0], sizeof(left_R_data));
		memcpy(&left_cam_info_.P[0], &left_P_data[0], sizeof(left_P_data));
		memcpy(&right_cam_info_.D[0], &right_D_data[0], sizeof(right_D_data));
		memcpy(&right_cam_info_.K[0], &right_K_data[0], sizeof(right_K_data));
		memcpy(&right_cam_info_.R[0], &right_R_data[0], sizeof(right_R_data));
		memcpy(&right_cam_info_.P[0], &right_P_data[0], sizeof(right_P_data));
	}
};
// end Bumblebee2Node class definition

// global variables
Bumblebee2Node *g_cm;
int32_t g_runLevel = dynamic_reconfigure::SensorLevels::RECONFIGURE_CLOSE;

/** Segfault signal handler */
void sigsegv_handler(int sig) {
	signal(SIGSEGV, SIG_DFL);
	fprintf(stderr, "Segmentation fault, stopping camera driver.\n");
	g_cm->closeCamera();
	g_runLevel = dynamic_reconfigure::SensorLevels::RECONFIGURE_CLOSE;
	ROS_ERROR("Segmentation fault, stopping camera.");
	signal(SIGSEGV, &sigsegv_handler);
}

//* Dynamic reconfigure callback */
void reconfig(bumblebee2::Bumblebee2Config &config, uint32_t level) {
	ROS_INFO_STREAM("Reconfigure request: brightness " << config.brightness << ", encoding " << config.encoding << ", exposure " << config.exposure << ", gain " << config.gain << ", shutter " << config.shutter << ", whitebalance " << config.whitebalance);

	ROS_INFO_STREAM("reconfigure level = " << level);

	// TODO: check parameter values
	if (config.whitebalance == "")
		config.whitebalance = "auto";

	ROS_DEBUG_STREAM("Reconfigured to: brightness " << config.brightness << ", encoding " << config.encoding << ", exposure " << config.exposure << ", gain " << config.gain << ", shutter " << config.shutter << ", whitebalance " << config.whitebalance);
}

/** Main entry point */
int main(int argc, char **argv) {
	ros::init(argc, argv, "bumblebee2");
	ros::NodeHandle node;

	dynamic_reconfigure::Server<bumblebee2::Bumblebee2Config> srv;
	dynamic_reconfigure::Server<bumblebee2::Bumblebee2Config>::CallbackType f = boost::bind(&reconfig, _1, _2);
	srv.setCallback(f);

	g_cm = new Bumblebee2Node();
	signal(SIGSEGV, &sigsegv_handler);

	// node may stay up without camera open to allow parameter
	// configuration while not running
	if (g_cm->openCamera()) {
		g_runLevel = dynamic_reconfigure::SensorLevels::RECONFIGURE_RUNNING;
	}

	while (node.ok()) {
		if (g_runLevel == dynamic_reconfigure::SensorLevels::RECONFIGURE_RUNNING) {
			g_cm->getParameters(); // check reconfigurable parameters
			g_cm->read(); // get camera data
		}

		ros::spinOnce();
	}

	g_cm->closeCamera();
	delete g_cm;

	return 0;
}

