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
 * @file retinal_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Sep 28, 2012
 *
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat retina_parvo_;
cv::Mat retina_magno_;
cv::Ptr<cv::Retina> retina_;

sensor_msgs::Image image_;

image_transport::Publisher parvo_image_pub_;
image_transport::Publisher parvo_image_mono_pub_;
image_transport::Publisher magno_image_pub_;

void callback(const sensor_msgs::ImageConstPtr& msgs)
{
	cv_bridge::CvImageConstPtr cp = cv_bridge::toCvShare(msgs, "rgb8");

	if(retina_==0) {
		//allocate "classical" retina :
		retina_ = new cv::Retina(cp->image.size());
		// load parameters if file exists
		retina_->setup("RetinaSpecificParameters.xml");
		retina_->clearBuffers();
	}

	retina_->run(cp->image);
	// Retrieve and display retina output
	retina_->getParvo(retina_parvo_);
	retina_->getMagno(retina_magno_);

	cv_bridge::CvImage im_parvo(msgs->header, "rgb8", retina_parvo_);
	parvo_image_pub_.publish(im_parvo.toImageMsg());

	cv::Mat gray;
	cv::cvtColor(retina_parvo_, gray, CV_BGR2GRAY);
	cv_bridge::CvImage im_parvo_mono(msgs->header, "mono8", gray);
	parvo_image_mono_pub_.publish(im_parvo_mono.toImageMsg());

	//msgs->header.encoding = "mono";
	cv_bridge::CvImage im_magno(msgs->header, "mono8", retina_magno_);
	magno_image_pub_.publish(im_magno.toImageMsg());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "retinal_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	image_transport::ImageTransport it(nh);

	parvo_image_pub_ = it.advertise(nh_priv.getNamespace() + "/parvo/image", 1);
	parvo_image_mono_pub_ = it.advertise(nh_priv.getNamespace() + "/parvo/image_mono", 1);
	magno_image_pub_ = it.advertise(nh_priv.getNamespace() + "/magno/image", 1);

    ros::Subscriber sub = nh.subscribe("image", 1, callback);

    retina_ = 0;

    ros::spin();
}
