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
 * @file region_growing_nodelet.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Dec 2, 2013
 *
 */
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "lrm_stereo/zigzag_cut.h"

namespace lrm_stereo
{

class ZigzagCutNodelet: public nodelet::Nodelet
{
public:
	virtual void onInit()
	{
		ROS_WARN("zigzag cut nodelet init");
		server_.reset(new ZigzagCutFilter(
				this->getNodeHandle(),
				this->getPrivateNodeHandle()));
	}

private:
	boost::shared_ptr<ZigzagCutFilter> server_;

};

}
// Register nodelet
PLUGINLIB_EXPORT_CLASS(lrm_stereo::ZigzagCutNodelet, nodelet::Nodelet)
