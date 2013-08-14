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
 * @file GPSPlugin.h
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Dec 15, 2012
 *
 */

#ifndef GPSPLUGIN_H_
#define GPSPLUGIN_H_

#include <boost/bind.hpp>
//#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>

#include <ros/ros.h>

namespace gazebo
{
class GPSPlugin: public ModelPlugin
{

public:
	GPSPlugin();
	~GPSPlugin();
	void Load(physics::ModelPtr, sdf::ElementPtr);
	void OnUpdate();
	//void ROSCallback(const std_msgs::Float64::ConstPtr& msg);

private:
	physics::ModelPtr _model;
	// Pointer to the update event connection
	event::ConnectionPtr _updateConnection;
	ros::NodeHandle* _node;
	ros::Subscriber _sub;

	double _latitude;
	double _longitude;
	double _elevation;
};

}

#endif /* GPSPLUGIN_H_ */
