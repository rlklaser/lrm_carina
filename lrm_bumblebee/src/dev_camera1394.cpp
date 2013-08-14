// $Id: dev_camera1394.cpp 28709 2010-04-10 00:37:52Z joq $

///////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2009, 2010 Patrick Beeson
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

// Copyright (C) 2010 Soonhac Hong
// Add Bumblebee2 into player

#include <stdint.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include "bumblebee2.h"

namespace enc = sensor_msgs::image_encodings;

#define NUM_DMA_BUFFERS 4

const timespec NSLEEP_TIME = { 0, 10000 }; // (0s, 10ms) => max 100Hz

//! Macro for throwing an exception with a message
#define CAM_EXCEPT(except, msg)					\
  {								\
    char buf[100];						\
    snprintf(buf, 100, "[Camera1394::%s]: " msg, __FUNCTION__); \
    throw except(buf);						\
  }

//! Macro for throwing an exception with a message, passing args
#define CAM_EXCEPT_ARGS(except, msg, ...)				\
  {									\
    char buf[100];							\
    snprintf(buf, 100, "[Camera1394::%s]: " msg, __FUNCTION__, __VA_ARGS__); \
    throw except(buf);							\
  }

using namespace bumblebee2;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Camera1394::Camera1394() :
		camera_(NULL) {
}

Camera1394::~Camera1394() {
	SafeCleanup();
}

void Camera1394::findFrameRate(float fps) {
	if (fps < 3.75)
		frameRate_ = DC1394_FRAMERATE_1_875;
	else if (fps < 7.5)
		frameRate_ = DC1394_FRAMERATE_3_75;
	else if (fps < 15)
		frameRate_ = DC1394_FRAMERATE_7_5;
	else if (fps < 30)
		frameRate_ = DC1394_FRAMERATE_15;
	else if (fps < 60)
		frameRate_ = DC1394_FRAMERATE_30;
	else
		frameRate_ = DC1394_FRAMERATE_60;
}

void Camera1394::findVideoMode(const char* mode) {
	if (0 == strcmp(mode, "320x240_yuv422"))
		videoMode_ = DC1394_VIDEO_MODE_320x240_YUV422;
	else if (0 == strcmp(mode, "640x480_mono"))
		videoMode_ = DC1394_VIDEO_MODE_640x480_MONO8;
	else if (0 == strcmp(mode, "640x480_yuv422"))
		videoMode_ = DC1394_VIDEO_MODE_640x480_YUV422;
	else if (0 == strcmp(mode, "640x480_rgb"))
		videoMode_ = DC1394_VIDEO_MODE_640x480_RGB8;
	else if (0 == strcmp(mode, "640x480_stereo_mono")) // added by soonhac for bumblebee2
		videoMode_ = DC1394_VIDEO_MODE_640x480_MONO16;
	else if (0 == strcmp(mode, "800x600_mono"))
		videoMode_ = DC1394_VIDEO_MODE_800x600_MONO8;
	else if (0 == strcmp(mode, "800x600_yuv422"))
		videoMode_ = DC1394_VIDEO_MODE_800x600_YUV422;
	else if (0 == strcmp(mode, "1024x768_mono"))
		videoMode_ = DC1394_VIDEO_MODE_1024x768_MONO8;
	else if (0 == strcmp(mode, "1024x768_yuv422"))
		videoMode_ = DC1394_VIDEO_MODE_1024x768_YUV422;
	else if (0 == strcmp(mode, "1280x960_mono"))
		videoMode_ = DC1394_VIDEO_MODE_1280x960_MONO8;
	else
		//if (0 == strcmp(mode, "1280x960_yuv422")) {
		videoMode_ = DC1394_VIDEO_MODE_1280x960_YUV422;
	//}
}

void Camera1394::findBayerFilter(const char* bayer, const char* method) {
	DoBayerConversion_ = false;

	if (strcmp(bayer, "NONE") && strcmp(method, "NONE")) {
		if (!strcmp(bayer, "BGGR")) {
			DoBayerConversion_ = true;
			BayerPattern_ = DC1394_COLOR_FILTER_BGGR;
		} else if (!strcmp(bayer, "GRBG")) {
			DoBayerConversion_ = true;
			BayerPattern_ = DC1394_COLOR_FILTER_GRBG;
		} else if (!strcmp(bayer, "RGGB")) {
			DoBayerConversion_ = true;
			BayerPattern_ = DC1394_COLOR_FILTER_RGGB;
		} else if (!strcmp(bayer, "GBRG")) {
			DoBayerConversion_ = true;
			BayerPattern_ = DC1394_COLOR_FILTER_GBRG;
		} else {
			ROS_ERROR("unknown bayer pattern [%s]", bayer);
		}
	}

	// Set default Bayer decoding method
	if (DoBayerConversion_)
		BayerMethod_ = DC1394_BAYER_METHOD_SIMPLE;

	// Check for user selected method
	if (strcmp(method, "NONE")) {
		if (!DoBayerConversion_)
			ROS_ERROR("bayer method [%s] specified without enabling bayer conversion", method);
		else {
			if (!strcmp(method, "DownSample")) {
				BayerMethod_ = DC1394_BAYER_METHOD_DOWNSAMPLE;
			} else if (!strcmp(method, "Simple")) {
				BayerMethod_ = DC1394_BAYER_METHOD_SIMPLE;
//       } else if (!strcmp(method, "Edge")) {
// 	BayerMethod_ = DC1394_BAYER_METHOD_EDGESENSE;
			} else if (!strcmp(method, "Bilinear")) {
				BayerMethod_ = DC1394_BAYER_METHOD_BILINEAR;
			} else if (!strcmp(method, "HQ")) {
				BayerMethod_ = DC1394_BAYER_METHOD_HQLINEAR;
			} else if (!strcmp(method, "VNG")) {
				BayerMethod_ = DC1394_BAYER_METHOD_VNG;
			} else if (!strcmp(method, "AHD")) {
				BayerMethod_ = DC1394_BAYER_METHOD_AHD;
			} else {
				ROS_ERROR("unknown bayer method [%s].  Defaulting to DownSample.", method);
			}
		}
	}
}

void Camera1394::findIsoSpeed(int iso_speed) {
	switch (iso_speed) {
	case 100:
		isoSpeed_ = DC1394_ISO_SPEED_100;
		break;
	case 200:
		isoSpeed_ = DC1394_ISO_SPEED_200;
		break;
	case 400:
		isoSpeed_ = DC1394_ISO_SPEED_400;
		break;
	case 800:
		isoSpeed_ = DC1394_ISO_SPEED_800;
		break;
	case 1600:
		isoSpeed_ = DC1394_ISO_SPEED_1600;
		break;
	case 3200:
		isoSpeed_ = DC1394_ISO_SPEED_3200;
		break;
	default:
		ROS_ERROR("Unsupported iso_speed. Defaulting to 400.");
		isoSpeed_ = DC1394_ISO_SPEED_400;
		break;
	}
}

/** Open the 1394 device */
int Camera1394::open(const char* guid, const char* video_mode, float fps, int iso_speed, const char* bayer, const char* method) {

	findFrameRate(fps);
	findVideoMode(video_mode);
	findBayerFilter(bayer, method);
	findIsoSpeed(iso_speed);

	// First we try to find a camera
	int err;
	dc1394_t *d;
	dc1394camera_list_t *list;

	// TODO: make error exit paths clean up resources properly
	d = dc1394_new();
	if (d == NULL) {
		CAM_EXCEPT(bumblebee2::Exception, "Could not initialize dc1394_context.\n"
		"Make sure /dev/raw1394 exists, you have access permission,\n"
		"and libraw1394 development package is installed.");
	}

	err = dc1394_camera_enumerate(d, &list);
	if (err != DC1394_SUCCESS) {
		CAM_EXCEPT(bumblebee2::Exception, "Could not get camera list");
		return -1;
	}

	if (list->num == 0) {
		CAM_EXCEPT(bumblebee2::Exception, "No cameras found");
		return -1;
	}

	char* temp = (char*) malloc(1024 * sizeof(char));
	for (unsigned i = 0; i < list->num; i++) {
		// Create a camera
		camera_ = dc1394_camera_new(d, list->ids[i].guid);
		if (!camera_)
			ROS_WARN_STREAM("Failed to initialize camera with GUID " << std::hex << list->ids[i].guid);
		else
			ROS_INFO_STREAM("Found camera with GUID " << std::hex << list->ids[i].guid);

		uint32_t value[3];

		value[0] = camera_->guid & 0xffffffff;
		value[1] = (camera_->guid >> 32) & 0x000000ff;
		value[2] = (camera_->guid >> 40) & 0xfffff;

		sprintf(temp, "%06x%02x%08x", value[2], value[1], value[0]);

		if (strcmp(guid, "NONE") == 0) {
			ROS_INFO_STREAM("No guid specified, using first camera found, GUID: " << std::hex << camera_->guid);
			device_id_ = std::string(temp);
			break;
		}

		ROS_DEBUG("Comparing %s to %s", guid, temp);
		if (strcmp(temp, guid) == 0) {
			device_id_ = guid;
			break;
		}
		SafeCleanup();
	}
	free(temp);
	dc1394_camera_free_list(list);

	if (!camera_) {
		if (strcmp(guid, "NONE") == 0) {
			CAM_EXCEPT(bumblebee2::Exception, "Could not find camera");
		} else {
			CAM_EXCEPT_ARGS(bumblebee2::Exception, "Could not find camera with guid %s", guid);
		}
		return -1;
	}

	if (DC1394_SUCCESS != dc1394_camera_reset(camera_)) {
		SafeCleanup();
		CAM_EXCEPT(bumblebee2::Exception, "Unable to reset camera.");
		return -1;
	}

	// Get the ISO channel and speed of the video

	dc1394speed_t speed;
	if (DC1394_SUCCESS != dc1394_video_get_iso_speed(camera_, &speed)) {
		SafeCleanup();
		CAM_EXCEPT(bumblebee2::Exception, "Unable to get iso data; is the camera plugged in?");
		return -1;
	}

	// Set camera to use DMA, improves performance.
	bool DMA_Success = true;
	// first set parameters that are common between format 7 and other modes
	if (DC1394_SUCCESS != dc1394_video_set_framerate(camera_, frameRate_)) {
		ROS_WARN("Failed to set frameRate");
		DMA_Success = false;
	}
	if (DC1394_SUCCESS != dc1394_video_set_iso_speed(camera_, isoSpeed_)) {
		ROS_WARN("Failed to set iso speed");
		DMA_Success = false;
	}
	if (DC1394_SUCCESS != dc1394_video_set_mode(camera_, videoMode_)) {
		ROS_WARN("Failed to set mode");
		DMA_Success = false;
	}

	// now start capture
	if (DC1394_SUCCESS != dc1394_capture_setup(camera_, NUM_DMA_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT))
		DMA_Success = false;

	if (!DMA_Success) {
		SafeCleanup();
		CAM_EXCEPT(bumblebee2::Exception, "Failed to open device!");
		return -1;
	}

	// Start transmitting camera data
	if (DC1394_SUCCESS != dc1394_video_set_transmission(camera_, DC1394_ON)) {
		SafeCleanup();
		CAM_EXCEPT(bumblebee2::Exception, "Failed to start device!");
		return -1;
	}

	return 0;

}

/** Safe Cleanup */
void Camera1394::SafeCleanup() {
	if (camera_) {
		dc1394_capture_stop(camera_);
		dc1394_camera_free(camera_);
	}
	camera_ = NULL;

	// Also clean up data if memory is allocated later.
}

#if 0
int Camera1394::setZoom(unsigned int zoom)
{
	if (DC1394_SUCCESS !=
			dc1394_feature_set_value(camera_, DC1394_FEATURE_ZOOM, zoom))
	{
		ROS_ERROR("Unable to set Zoom value");
		return -1;
	}
	return 0;
}

int Camera1394::GetZoom(unsigned int* zoom)
{
	if (DC1394_SUCCESS !=
			dc1394_feature_get_value(camera_, DC1394_FEATURE_ZOOM, zoom))
	{
		ROS_ERROR("Unable to get Zoom value");
		return -1;
	}
	return 0;
}

int Camera1394::setFocus(int focus)
{
	bool autoFocus = focus < 0;

	if (DC1394_SUCCESS !=
			dc1394_feature_set_mode(camera_, DC1394_FEATURE_FOCUS,
					(autoFocus?
							DC1394_FEATURE_MODE_AUTO:
							DC1394_FEATURE_MODE_MANUAL)))
	{
		ROS_ERROR("Unable to set Focus mode");
		SafeCleanup();
		return -1;
	}

	if (!autoFocus)
	{
		if (DC1394_SUCCESS !=
				dc1394_feature_set_value(camera_, DC1394_FEATURE_FOCUS, focus))
		{
			ROS_ERROR("Unable to set Focus value");
			return -1;
		}
	}
	return 0;
}

int Camera1394::GetFocus(unsigned int* focus)
{
	if (DC1394_SUCCESS !=
			dc1394_feature_get_value(camera_, DC1394_FEATURE_FOCUS, focus))
	{
		ROS_ERROR("Unable to get Focus value");
		return -1;
	}
	return 0;
}

int Camera1394::setIris(int iris)
{
	bool autoIris = iris < 0;
	if (DC1394_SUCCESS !=
			dc1394_feature_set_mode(camera_, DC1394_FEATURE_IRIS,
					(autoIris?
							DC1394_FEATURE_MODE_AUTO:
							DC1394_FEATURE_MODE_MANUAL)))
	{
		ROS_ERROR("Unable to set Iris mode");
		SafeCleanup();
		return -1;
	}

	if (!autoIris)
	{

		if (DC1394_SUCCESS !=
				dc1394_feature_set_value(camera_, DC1394_FEATURE_IRIS, iris))
		{
			ROS_ERROR("Unable to set Iris value");
			return -1;
		}
	}
	return 0;
}

int Camera1394::GetIris(unsigned int* iris)
{
	if (DC1394_SUCCESS !=
			dc1394_feature_get_value(camera_, DC1394_FEATURE_IRIS, iris))
	{
		ROS_ERROR("Unable to set Iris value");
		return -1;
	}
	return 0;
}
#endif

int Camera1394::setBrightness(unsigned int brightness) {
	if (DC1394_SUCCESS != dc1394_feature_set_value(camera_, DC1394_FEATURE_BRIGHTNESS, brightness)) {
		ROS_ERROR("Unable to set Brightness value");
		return -1;
	}
	return 0;
}

int Camera1394::setExposure(unsigned int exposure) {
	if (DC1394_SUCCESS != dc1394_feature_set_value(camera_, DC1394_FEATURE_EXPOSURE, exposure)) {
		ROS_ERROR("Unable to set Exposure value");
		return -1;
	}
	return 0;
}

int Camera1394::setShutter(int shutter) {
	bool autoShutter = shutter < 0;

	if (DC1394_SUCCESS != dc1394_feature_set_mode(camera_, DC1394_FEATURE_SHUTTER, (autoShutter ? DC1394_FEATURE_MODE_AUTO : DC1394_FEATURE_MODE_MANUAL))) {
		ROS_ERROR("Unable to set Shutter mode");
		SafeCleanup();
		return -1;
	}

	if (!autoShutter) {
		if (DC1394_SUCCESS != dc1394_feature_set_value(camera_, DC1394_FEATURE_SHUTTER, shutter)) {
			ROS_ERROR("Unable to set Shutter value");
			return -1;
		}
	}
	return 0;
}

int Camera1394::setGain(int gain) {
	bool autoGain = gain < 0;

	if (DC1394_SUCCESS != dc1394_feature_set_mode(camera_, DC1394_FEATURE_GAIN, (autoGain ? DC1394_FEATURE_MODE_AUTO : DC1394_FEATURE_MODE_MANUAL))) {
		ROS_ERROR("Unable to set Gain mode");
		SafeCleanup();
		return -1;
	}

	if (!autoGain) {
		if (DC1394_SUCCESS != dc1394_feature_set_value(camera_, DC1394_FEATURE_GAIN, gain)) {
			ROS_ERROR("Unable to Gain value");
			SafeCleanup();
			return -1;
		}
	}
	return 0;
}

int Camera1394::setWhiteBalance(const char* whitebalance) {
	bool setWhiteBalance = false;
	bool autoWhiteBalance = false;
	unsigned int blueBalance, redBalance;

	if (!strcmp(whitebalance, "auto")) {
		setWhiteBalance = true;
		autoWhiteBalance = true;
	} else {
		autoWhiteBalance = false;
		if (sscanf(whitebalance, "%u %u", &blueBalance, &redBalance) == 2)
			setWhiteBalance = true;
		else
			ROS_ERROR("didn't understand white balance values [%s]", whitebalance);
	}

	if (setWhiteBalance) {
		if (DC1394_SUCCESS != dc1394_feature_set_mode(camera_, DC1394_FEATURE_WHITE_BALANCE, (autoWhiteBalance ? DC1394_FEATURE_MODE_AUTO : DC1394_FEATURE_MODE_MANUAL))) {
			ROS_ERROR("Unable to set White Balance mode");
			SafeCleanup();
			return -1;
		}

		if (!autoWhiteBalance)
			if (DC1394_SUCCESS != dc1394_feature_whitebalance_set_value(camera_, blueBalance, redBalance)) {
				ROS_ERROR("Unable to set White Balance");
				SafeCleanup();
				return -1;
			}
	}
	return 0;
}

/** close the 1394 device */
int Camera1394::close() {
	if (camera_) {
		if (DC1394_SUCCESS != dc1394_video_set_transmission(camera_, DC1394_OFF) || DC1394_SUCCESS != dc1394_capture_stop(camera_))
			ROS_WARN("unable to stop camera");
	}

	// Free resources
	SafeCleanup();

	return 0;
}

/** Return an image frame */
void Camera1394::readData(sensor_msgs::Image& image) {

	if (camera_ == NULL) {
		CAM_EXCEPT(bumblebee2::Exception, "Read attempted on NULL camera port!");
		return;
	}

	//  CAM_EXCEPT(bumblebee2::Exception, "Read not implemented.");

	dc1394video_frame_t * frame = NULL;

	dc1394_capture_dequeue(camera_, DC1394_CAPTURE_POLICY_WAIT, &frame);
	if (!frame) {
		CAM_EXCEPT(bumblebee2::Exception, "Unable to capture frame");
		return;
	}

	uint8_t* capture_buffer;

	image.header.stamp = ros::Time((double) frame->timestamp / 1000000.0);

	if (DoBayerConversion_) {
		dc1394video_frame_t frame2;

		frame->color_filter = BayerPattern_;
		// debayer frame into RGB8
		frame2.image = (unsigned char *) malloc(frame->size[0] * frame->size[1] * 3 * sizeof(unsigned char));
		frame2.color_coding = DC1394_COLOR_CODING_RGB8;
		int err = dc1394_debayer_frames(frame, &frame2, BayerMethod_);

		if (err != DC1394_SUCCESS)
			ROS_ERROR("Could not convert/debayer frames");

		capture_buffer = reinterpret_cast<uint8_t *>(frame2.image);

		image.header.stamp = ros::Time(frame2.timestamp * 1.e-6);
		image.width = frame2.size[0];
		image.height = frame2.size[1];
	} else {
		image.header.stamp = ros::Time(frame->timestamp * 1.e-6);
		image.width = frame->size[0];
		image.height = frame->size[1];
		capture_buffer = reinterpret_cast<uint8_t *>(frame->image);
	}

	assert(capture_buffer);

	int image_size;
	switch (videoMode_) {
	case DC1394_VIDEO_MODE_320x240_YUV422:
	case DC1394_VIDEO_MODE_640x480_YUV422:
	case DC1394_VIDEO_MODE_800x600_YUV422:
	case DC1394_VIDEO_MODE_1024x768_YUV422:
	case DC1394_VIDEO_MODE_1280x960_YUV422:
		image.step = image.width * 3;
		image_size = image.height * image.step;
		image.encoding = "rgb8";
		image.data.resize(image_size);
		uyvy2rgb(reinterpret_cast<unsigned char *>(capture_buffer), reinterpret_cast<unsigned char *>(&image.data[0]), image.width * image.height);
		break;
	case DC1394_VIDEO_MODE_640x480_RGB8:
		image.step = image.width * 3;
		image_size = image.height * image.step;
		image.encoding = "rgb8";
		image.data.resize(image_size);
		memcpy(&image.data[0], capture_buffer, image_size);
		break;
	case DC1394_VIDEO_MODE_640x480_MONO8:
	case DC1394_VIDEO_MODE_800x600_MONO8:
	case DC1394_VIDEO_MODE_1024x768_MONO8:
	case DC1394_VIDEO_MODE_1280x960_MONO8:
		if (!DoBayerConversion_) {
			image.step = image.width;
			image_size = image.height * image.step;
			image.encoding = "mono8";
			image.data.resize(image_size);
			memcpy(&image.data[0], capture_buffer, image_size);
		} else {
			image.step = image.width * 3;
			image_size = image.height * image.step;
			image.encoding = "rgb8";
			image.data.resize(image_size);
			memcpy(&image.data[0], capture_buffer, image_size);
		}
		break;
	case DC1394_VIDEO_MODE_640x480_MONO16: // added by soonhac for bumblebee2
		image.step = image.width;
		image_size = image.height * image.step * 2; //image size should be double size of image size because each pixel has 16 bit data.
		image.encoding = "mono8";
		image.data.resize(image_size);
		memcpy(&image.data[0], capture_buffer, image_size);
		//capture_buffer stores MONO 16bit data : MSB - right camera, left -right camera
		//Deinterlace caputre_buffer into deintrelaced image which consists of 8bit right image and 8bit left image
		if (DC1394_SUCCESS != dc1394_deinterlace_stereo(reinterpret_cast<unsigned char *>(capture_buffer), reinterpret_cast<unsigned char *>(&image.data[0]), image.width, image.height * 2)) {
			ROS_ERROR("Unable to deinterlace stereo image");
			SafeCleanup();
			return;
		}
		break;
		break;
	default:
		CAM_EXCEPT(bumblebee2::Exception, "Unknown image mode");
		return;
	}
	dc1394_capture_enqueue(camera_, frame);

	if (DoBayerConversion_)
		free(capture_buffer);
}

void Camera1394::uyvy2rgb(unsigned char *src, unsigned char *dest, unsigned long long int NumPixels) {
	register int i = (NumPixels << 1) - 1;
	register int j = NumPixels + (NumPixels << 1) - 1;
	register int y0, y1, u, v;
	register int r, g, b;

	while (i > 0) {
		y1 = (unsigned char) src[i--];
		v = (unsigned char) src[i--] - 128;
		y0 = (unsigned char) src[i--];
		u = (unsigned char) src[i--] - 128;
		YUV2RGB(y1, u, v, r, g, b);
		dest[j--] = b;
		dest[j--] = g;
		dest[j--] = r;
		YUV2RGB(y0, u, v, r, g, b);
		dest[j--] = b;
		dest[j--] = g;
		dest[j--] = r;
	}
}

