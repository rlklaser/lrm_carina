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
 * @file CaRINAGazeboPlugin.h
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Aug 24, 2013
 *
 */

#ifndef CARINAGAZEBOPLUGIN_H_
#define CARINAGAZEBOPLUGIN_H_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>

namespace gazebo
{
class CaRINAGazeboPlugin: public ModelPlugin
{
private:

	physics::ModelPtr _model;
	ros::NodeHandle* _node;
	ros::Subscriber _sub;

	void SetupSteeringConstraint();

public:

	CaRINAGazeboPlugin();
	virtual ~CaRINAGazeboPlugin();
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	void OnUpdate();
};

}

#endif /* CARINAGAZEBOPLUGIN_H_ */
