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
 * @file flatten_ground_nodelet.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Dec 2, 2013
 *
 */
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "lrm_stereo/flatten_ground.h"

namespace lrm_stereo
{

class FlattenGroundNodelet: public nodelet::Nodelet
{
public:
	virtual void onInit();
private:
	boost::shared_ptr<FlattenGroundFilter> server_;

};

void FlattenGroundNodelet::onInit()
{
	ROS_WARN("flatten ground nodelet init");
	ros::NodeHandle& nh = getNodeHandle();
	ros::NodeHandle& private_nh = getPrivateNodeHandle();

	server_.reset(new FlattenGroundFilter(nh, private_nh));
}

}
// Register nodelet
PLUGINLIB_EXPORT_CLASS(lrm_stereo::FlattenGroundNodelet, nodelet::Nodelet)
