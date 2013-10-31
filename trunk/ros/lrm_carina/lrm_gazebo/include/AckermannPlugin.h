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
 * @file AckermannPlugin.h
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 13, 2013
 *
 */

#ifndef ACKERMANNPLUGIN_H_
#define ACKERMANNPLUGIN_H_

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>
//#include <sdf/sdf.hh>

namespace gazebo {

class AckermannPlugin: public ModelPlugin {

public:
	AckermannPlugin();
	virtual ~AckermannPlugin();
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	void Init();

protected:
	void OnUpdate();

private:
	physics::ModelPtr _model;
	event::ConnectionPtr _updateConnection;
	ros::NodeHandle* _node;
	ros::Subscriber _sub;

};

}

#endif /* ACKERMANNPLUGIN_H_ */
