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
 * @file CarinaModelPlugin.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Oct 9, 2013
 *
 */

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <ros/ros.h>

namespace gazebo
{
  class CarinaModelPlugin : public ModelPlugin
  {

  public:

	  CarinaModelPlugin() {}

	  ~CarinaModelPlugin() {
		  delete node_;
	  }

	  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
	  		// Store the pointer to the model
	  		model_ = _parent;

	  		// ROS Nodehandle
	  		node_ = new ros::NodeHandle("~");
	  }

	  void OnUpdate() {
	  		ros::spinOnce();
	  }

  private:
  	physics::ModelPtr model_;
  	ros::NodeHandle* node_;
  	ros::Subscriber sub_;

  }

};

GZ_REGISTER_MODEL_PLUGIN(CarinaModelPlugin);
