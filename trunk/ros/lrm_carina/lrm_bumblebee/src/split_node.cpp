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
 * @file split_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Oct 5, 2012
 *
 */
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

sensor_msgs::Image        left_img_;
sensor_msgs::Image        right_img_;
sensor_msgs::CameraInfo   left_cam_info_;
sensor_msgs::CameraInfo   right_cam_info_;
std_msgs::String		  calibration_settings_;

image_transport::CameraPublisher left_cam_pub_;
image_transport::CameraPublisher right_cam_pub_;

std::string p_camera_left_prefix;
std::string p_camera_right_prefix;

bool fillImages(sensor_msgs::Image& left_image, sensor_msgs::Image& right_image,
		std::string encoding_arg,
		uint32_t rows_arg,
		uint32_t cols_arg,
		uint32_t step_arg,
		const void* data_arg)
{
	size_t st0 = (step_arg * rows_arg);
	left_image.encoding = encoding_arg;
	left_image.height   = rows_arg;
	left_image.width    = cols_arg;
	left_image.step     = cols_arg; //step_arg;
	left_image.data.resize(st0);

	right_image.encoding = encoding_arg;
	right_image.height   = rows_arg;
	right_image.width    = cols_arg;
	right_image.step     = cols_arg; //step_arg;
	right_image.data.resize(st0);

#if API

	unsigned char* pucGrabBuffer = data_arg;
	dc1394bayer_method_t bayerMethod = DC1394_BAYER_METHOD_VNG;
	dc1394color_filter_t bayerTile DC1394_COLOR_FILTER_RGGB;

	unsigned int nBufferSize = rows_arg * cols_arg * 16;
	// allocate a buffer to hold the de-interleaved images
	unsigned char* pucDeInterleaved = new unsigned char[nBufferSize];
	unsigned char* pucRGB = new unsigned char[3 * nBufferSize];

	// de-interlace the 16 bit data into 2 bayer tile pattern images
	dc1394_deinterlace_stereo(pucGrabBuffer, pucDeInterleaved, cols_arg, 2 * rows_arg);
	// extract color from the bayer tile image
	// note: this will alias colors on the top and bottom rows
	dc1394_bayer_decoding_8bit(pucDeInterleaved, pucRGB, cols_arg, 2 * rows_arg, bayerTile, bayerMethod);
	// now deinterlace the RGB Buffer to extract the green channel
	// The green channel is a quick and dirty approximation to the mono
	// equivalent of the image and can be used for stereo processing
	int sz = 3 * cols_arg * rows_arg;
	//*ppucRightRGB = pucRGB;
	//*ppucLeftRGB = pucRGB + 3 * stereoCamera->nRows * stereoCamera->nCols;
	memcpy(&right_image.data[0], pucRGB, sz);
	memcpy(&left_image.data[0], pucRGB+sz, sz);

#else

	char* crt = (char*) data_arg;
	//char* end = crt + 3*st0;
	char* left = (char*)&left_image.data[0];
	char* right = (char*)&right_image.data[0];

	int k = 320 * 240;

	//NO DEBAYERING!!!

	#pragma omp parallel while
	while(k>0) {

		//std::cout << k << std::endl;

		*left = *crt;
		*right = *(crt+1);

		crt+=2;
		right++;
		left++;

		*left = *crt;
		*right = *(crt+1);

		crt+=2;
		right++;
		left++;

		*left = *crt;
		*right = *(crt+1);

		crt+=2;
		right++;
		left++;

		*left = *crt;
		*right = *(crt+1);

		crt+=2;
		right++;
		left++;

		k--;
	}
    #endif

	left_image.is_bigendian = 0;
	right_image.is_bigendian = 0;
	return true;
}

bool fillImagesRGB(sensor_msgs::Image& left_image, sensor_msgs::Image& right_image,
		std::string encoding_arg,
		uint32_t rows_arg,
		uint32_t cols_arg,
		uint32_t step_arg,
		const void* data_arg)
{
	size_t st0 = (step_arg * rows_arg);
	left_image.encoding = encoding_arg;
	left_image.height   = rows_arg;
	left_image.width    = cols_arg;
	left_image.step     = cols_arg; //step_arg;
	left_image.data.resize(st0);

	right_image.encoding = encoding_arg;
	right_image.height   = rows_arg;
	right_image.width    = cols_arg;
	right_image.step     = cols_arg; //step_arg;
	right_image.data.resize(st0);

	char* crt = (char*) data_arg;
	//char* end = crt + 3*st0;
	char* left = (char*)&left_image.data[0];
	char* right = (char*)&right_image.data[0];

	int k = 320 * 240;

	//NO DEBAYERING!!!

	while(k>0) {

		//std::cout << k << std::endl;

		*left = *crt;
		*right = *(crt+1);

		crt+=2;
		right++;
		left++;

		*left = *crt;
		*right = *(crt+1);

		crt+=2;
		right++;
		left++;

		*left = *crt;
		*right = *(crt+1);

		crt+=2;
		right++;
		left++;

		*left = *crt;
		*right = *(crt+1);

		crt+=2;
		right++;
		left++;

		k--;
	}

	left_image.is_bigendian = 0;
	right_image.is_bigendian = 0;
	return true;
}

void publishCam(const sensor_msgs::Image& image)
{
	uint32_t imHeight = image.height;
	uint32_t imWidth = image.width;
	uint32_t imStep = imWidth;
	const void *imRaw = image.data.data();

	imStep = 480 * 4;

	std::string encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
	//std::string encoding = sensor_msgs::image_encodings::RGB8;

	fillImages(left_img_, right_img_, encoding, imHeight, imWidth, imStep, imRaw);
	//fillImagesRGB(left_img_, right_img_, encoding, imHeight, imWidth, imStep, imRaw);

	// Update diagnostics and publish
	ros::Time stamp = ros::Time::now();


	left_img_.header.frame_id="/camera";
	right_img_.header.frame_id="/camera";
	left_cam_info_.header.frame_id = left_img_.header.frame_id;
	left_cam_info_.header.stamp = stamp;

	left_cam_info_.height = imHeight;
	left_cam_info_.width  = imWidth;

	right_cam_info_.header.frame_id = right_img_.header.frame_id;
	right_cam_info_.header.stamp = stamp;

	right_cam_info_.height = imHeight;
	right_cam_info_.width  = imWidth;

	left_cam_pub_.publish(left_img_, left_cam_info_, stamp);
	right_cam_pub_.publish(right_img_, right_cam_info_, stamp);
}

void callback(const sensor_msgs::ImageConstPtr& msgs)
{
	publishCam(*msgs);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "split_node");
	ros::NodeHandle nh;

	ros::NodeHandle nh_priv("~");
	ros::NodeHandle nh_left("~/left");
	ros::NodeHandle nh_right("~/right");

	nh_priv.param("camera_left_prefix", p_camera_left_prefix, std::string("camera_left"));
	nh_priv.param("camera_right_prefix", p_camera_right_prefix, std::string("camera_right"));

	image_transport::ImageTransport it(nh);
	left_cam_pub_ = it.advertiseCamera(nh_priv.getNamespace() + "/left/image_raw", 1);
	right_cam_pub_ = it.advertiseCamera(nh_priv.getNamespace() + "/right/image_raw", 1);

	camera_info_manager::CameraInfoManager info_manager_left(nh_left, p_camera_left_prefix);
	left_cam_info_ = info_manager_left.getCameraInfo();

	camera_info_manager::CameraInfoManager info_manager_right(nh_right, p_camera_right_prefix);
	right_cam_info_ = info_manager_right.getCameraInfo();

	ros::Subscriber sub = nh.subscribe("/camera/image_raw", 1, &callback);

	ros::spin();
}
