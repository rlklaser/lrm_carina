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
 * @file AckermannPlugin.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 13, 2013
 *
 */

#include "AckermannPlugin.h"

#include <ode/ode.h>

namespace gazebo {

AckermannPlugin::AckermannPlugin() {
	// Start up ROS
	std::string name = "ackermann_plugin_node";
	int argc = 0;
	ros::init(argc, NULL, name);
}

AckermannPlugin::~AckermannPlugin() {
	this->_node->shutdown();
	delete this->_node;
}

void AckermannPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
	// Store the pointer to the model
	this->_model = _parent;

	// ROS Nodehandle
	this->_node = new ros::NodeHandle("~");

	//std::string ackermannLeft = _sdf->GetValueString("ackermannLeft");

	math::Vector3 anchor_l(0, 0.3, 0);
	math::Vector3 anchor_r(0, -0.3, 0);
	math::Vector3 axis(0, 0, 1);

	physics::LinkPtr l_susp = _model->GetLink("ackermann_left_susp_link");
	physics::LinkPtr r_susp = _model->GetLink("ackermann_right_susp_link");

	physics::LinkPtr l_bar = _model->GetLink("front_left_bar_link");
	physics::LinkPtr r_bar = _model->GetLink("front_right_bar_link");

	physics::JointPtr l_hinge;
	physics::JointPtr r_hinge;

	l_hinge = this->_model->GetWorld()->GetPhysicsEngine()->CreateJoint("revolute", this->_model);
	r_hinge = this->_model->GetWorld()->GetPhysicsEngine()->CreateJoint("revolute", this->_model);

	//std::cout << "l hinge:" << (l_hinge==0 ? "no" : "yes") << std::endl;
	//std::cout << "r hinge:" << (r_hinge==0 ? "no" : "yes") << std::endl;
	//std::cout << "r susp:" << (r_susp==0 ? "no" : "yes") << std::endl;
	//std::cout << "id:" << l_hinge->GetId() << std::endl;

	l_hinge->Attach(l_bar, l_susp);
	l_hinge->Load(l_bar, l_susp, math::Pose(anchor_l, math::Quaternion()));
	l_hinge->SetAxis(0, axis);
	l_hinge->SetHighStop(0, 2 * M_PI);
	l_hinge->SetLowStop(0, -2 * M_PI);

	//l_hinge->SetAttribute("cfm", 0, 0.2);
	//l_hinge->SetAttribute("erp", 0, 0.5);

	l_hinge->SetName("joint_ackermann_left_bar");
	l_hinge->Init();

	r_hinge->Attach(r_bar, r_susp);
	r_hinge->Load(r_bar, r_susp, math::Pose(anchor_r, math::Quaternion()));
	r_hinge->SetAxis(0, axis);
	r_hinge->SetHighStop(0, 2 * M_PI);
	r_hinge->SetLowStop(0, -2 * M_PI);

	//r_hinge->SetAttribute("cfm", 0, 0.2);
	//r_hinge->SetAttribute("erp", 0, 0.5);

	r_hinge->SetName("joint_ackermann_right_bar");
	r_hinge->Init();

}

void AckermannPlugin::Init() {
}

GZ_REGISTER_MODEL_PLUGIN(AckermannPlugin)
}
