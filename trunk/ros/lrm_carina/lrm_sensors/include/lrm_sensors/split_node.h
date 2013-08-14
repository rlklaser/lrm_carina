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
 * @file split_node.h
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jul 29, 2013
 *
 */

#ifndef SPLIT_NODE_H_
#define SPLIT_NODE_H_

#include <sensor_msgs/Image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/subscriber.h>



struct st_camera {
	sensor_msgs::Image image;
	sensor_msgs::CameraInfo info;
	image_transport::CameraPublisher publisher;
	boost::shared_ptr<camera_info_manager::CameraInfoManager> manager;
	std::string url;
};

struct st_stereo_camera {
	struct st_camera left;
	struct st_camera right;
	std::string frame_id;
	std::string name;
};

#endif /* SPLIT_NODE_H_ */
