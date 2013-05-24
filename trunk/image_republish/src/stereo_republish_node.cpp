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
 * @file camera_republish_node.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Sep 27, 2012
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
//#include <image_common/camera_info.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/distortion_models.h>

image_transport::ImageTransport *it_;
sensor_msgs::Image image_;
sensor_msgs::Image left_image_;
sensor_msgs::Image right_image_;
sensor_msgs::CameraInfo left_cam_info_;
sensor_msgs::CameraInfo right_cam_info_;
//CameraInfoManager *cinfo_;

/** image transport publish interface */
image_transport::CameraPublisher left_image_pub_;
image_transport::CameraPublisher right_image_pub_;

bool p_restamp;
std::string p_frame_id;

/** Update the bumblebee2 calibration data */
/** Author : Soonhac Hong (sh2723@columbia.edu) */
/** Date : 5/24/2010 */
/** Note : Calibration data is needed to show disparity image using image_view with stereo_view.*/
void updateBumblebee2CalibrationData()
{
	//needs -std=c++0x flag, {{}} = boost array initialization
	left_cam_info_.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
	//left_cam_info_.D = {-0.29496962080028677, 0.12120859315219049, -0.0019941265153862824, 0.0012058185627261283, 0.0};
	left_cam_info_.K = {{543.60636929659358, 0.0, 321.7411723319629, 0.0, 543.25622524820562, 268.04452669345528, 0.0, 0.0, 1.0}};
	left_cam_info_.R = {{0.99980275533925467, -0.018533834763323875, -0.0071377436911170388, 0.018542709766871161, 0.99982737377597841, 0.0011792212393866724, 0.0071146560377753926, -0.0013113417539480422, 0.9999738306837177}};
	left_cam_info_.P = {{514.20203529502226, 0.0, 334.37528610229492, 0.0, 0.0, 514.20203529502226, 268.46113204956055, 0.0, 0.0, 0.0, 1.0, 0.0}};

	right_cam_info_.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
	//right_cam_info_.D =  {-0.2893208200535437, 0.11215776927066376, -0.0003854904042866552, 0.00081197271575971614, 0.0};
	right_cam_info_.K =  {{541.66040340873735, 0.0, 331.73470962829737, 0.0, 541.60313005445187, 265.72960150703699, 0.0, 0.0, 1.0}};
	right_cam_info_.R =  {{0.99986888001551244, -0.012830354497672055, -0.0098795131453283894, 0.012818040762902759, 0.99991698911455085, -0.001308705884349387, 0.0098954841986237611, 0.001181898284639702, 0.99995034002140304}};
	right_cam_info_.P =  {{514.20203529502226, 0.0, 334.37528610229492, -232.44101555000066, 0.0, 514.20203529502226, 268.46113204956055, 0.0, 0.0, 0.0, 1.0, 0.0}};

}

void callback_left(const sensor_msgs::ImagePtr msgs)
{
	left_image_ = *msgs;

	if(p_restamp) {
		left_image_.header.stamp = ros::Time::now();
	}
	if(p_frame_id!="") {
		left_image_.header.frame_id = p_frame_id;
	}
	left_cam_info_.header.stamp = left_image_.header.stamp;
	left_cam_info_.header.frame_id = left_image_.header.frame_id;
	left_cam_info_.height =left_image_.height;
	left_cam_info_.width = left_image_.width;

	left_image_pub_.publish(left_image_, left_cam_info_);
}

void callback_right(const sensor_msgs::ImagePtr msgs)
{
	right_image_ = *msgs;

	if(p_restamp) {
		right_image_.header.stamp = ros::Time::now();
	}
	if(p_frame_id!="") {
		right_image_.header.frame_id = p_frame_id;
	}
	right_cam_info_.header.stamp = right_image_.header.stamp;
	right_cam_info_.header.frame_id = right_image_.header.frame_id;
	right_cam_info_.height =right_image_.height;
	right_cam_info_.width = right_image_.width;

	right_image_pub_.publish(right_image_, right_cam_info_);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stereo_republish_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	updateBumblebee2CalibrationData();

	//cinfo_ = new CameraInfoManager(privNH_, camera_name_);
	nh_priv.param<bool>("restamp", p_restamp, false);
	nh_priv.param<std::string>("frame_id", p_frame_id, "");

	it_ = new image_transport::ImageTransport(nh);
	left_image_pub_ = it_->advertiseCamera("left/image", 1);
	right_image_pub_ = it_->advertiseCamera("right/image", 1);

    ros::Subscriber left_sub = nh.subscribe("StereoCam/imageLeft", 1, callback_left);
    ros::Subscriber right_sub = nh.subscribe("StereoCam/imageRight", 1, callback_right);

    ros::spin();

}
