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
 * @file CaRINAGazeboPlugin.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Aug 24, 2013
 *
 */

//ackermann
//suspension
//encoder
//actuator
#include "lrm_gazebo_ros/CaRINAGazeboPlugin.h"

namespace gazebo
{

void CaRINAGazeboPlugin::SetupSteeringConstraint()
{
	math::Vector3 anchor_l(0, 0.3, 0);
	math::Vector3 anchor_r(0, -0.3, 0);
	math::Vector3 axis(0, 0, 1);

	physics::LinkPtr l_susp = _model->GetLink("ackermann_left_susp_link");
	physics::LinkPtr r_susp = _model->GetLink("ackermann_right_susp_link");

	physics::LinkPtr l_bar = _model->GetLink("front_left_bar_link");
	physics::LinkPtr r_bar = _model->GetLink("front_right_bar_link");

	physics::JointPtr l_hinge;
	physics::JointPtr r_hinge;

	l_hinge = _model->GetWorld()->GetPhysicsEngine()->CreateJoint("revolute", _model);
	r_hinge = _model->GetWorld()->GetPhysicsEngine()->CreateJoint("revolute", _model);

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

CaRINAGazeboPlugin::CaRINAGazeboPlugin()
{
}

CaRINAGazeboPlugin::~CaRINAGazeboPlugin()
{
	delete _node;
}

void CaRINAGazeboPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	// Store the pointer to the model
	_model = _parent;

	// ROS Nodehandle
	_node = new ros::NodeHandle("~");

	SetupSteeringConstraint();
}

void CaRINAGazeboPlugin::OnUpdate()
{
	ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN (CaRINAGazeboPlugin);
}
