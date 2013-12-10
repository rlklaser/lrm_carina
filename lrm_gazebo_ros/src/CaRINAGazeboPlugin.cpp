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

void CaRINAGazeboPlugin::SetupEncoders()
{
	_joint_wheel = _model->GetJoint("joint_front_right_wheel");
	_joint_steering = _model->GetJoint("joint_steer");
	_joint_steering_wheel = _model->GetJoint("joint_steering_wheel");

	_nh_priv->param("encoders_update_rate", _encoders_update_rate, 10.0);
	_nh_priv->param("encoders_topic", _encoders_topic, std::string("encoders"));

	_encoders_last_update = ros::Time::now();
	_encoders_acc_wheel = 0;
	_encoders_pub = _nh->advertise<lrm_msgs::Encoders>(_encoders_topic, 1);
}

void CaRINAGazeboPlugin::SetupSteeringConstraint()
{
	math::Vector3 anchor_l(0, 0.3, 0);
	math::Vector3 anchor_r(0, -0.3, 0);
	math::Vector3 axis(0, 0, 1);

	physics::LinkPtr l_susp = _model->GetLink("ackermann_left_susp_link");
	physics::LinkPtr r_susp = _model->GetLink("ackermann_right_susp_link");

	physics::LinkPtr l_bar = _model->GetLink("front_left_bar_link");
	physics::LinkPtr r_bar = _model->GetLink("front_right_bar_link");

	_l_hinge = _model->GetWorld()->GetPhysicsEngine()->CreateJoint("revolute", _model);
	_r_hinge = _model->GetWorld()->GetPhysicsEngine()->CreateJoint("revolute", _model);

	//std::cout << "l hinge:" << (l_hinge==0 ? "no" : "yes") << std::endl;
	//std::cout << "r hinge:" << (r_hinge==0 ? "no" : "yes") << std::endl;
	//std::cout << "r susp:" << (r_susp==0 ? "no" : "yes") << std::endl;
	//std::cout << "id:" << l_hinge->GetId() << std::endl;

	_l_hinge->Attach(l_bar, l_susp);
	_l_hinge->Load(l_bar, l_susp, math::Pose(anchor_l, math::Quaternion()));
	_l_hinge->SetAxis(0, axis);
	_l_hinge->SetHighStop(0, 2 * M_PI);
	_l_hinge->SetLowStop(0, -2 * M_PI);

	//l_hinge->SetAttribute("cfm", 0, 0.2);
	//l_hinge->SetAttribute("erp", 0, 0.5);

	_l_hinge->SetName("joint_ackermann_left_bar");
	_l_hinge->Init();

	_r_hinge->Attach(r_bar, r_susp);
	_r_hinge->Load(r_bar, r_susp, math::Pose(anchor_r, math::Quaternion()));
	_r_hinge->SetAxis(0, axis);
	_r_hinge->SetHighStop(0, 2 * M_PI);
	_r_hinge->SetLowStop(0, -2 * M_PI);

	//r_hinge->SetAttribute("cfm", 0, 0.2);
	//r_hinge->SetAttribute("erp", 0, 0.5);

	_r_hinge->SetName("joint_ackermann_right_bar");
	_r_hinge->Init();
}

CaRINAGazeboPlugin::CaRINAGazeboPlugin()
{
}

CaRINAGazeboPlugin::~CaRINAGazeboPlugin()
{
	//delete _node;
}

void CaRINAGazeboPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	// Store the pointer to the model
	_model = _parent;

	// ROS Nodehandle
	//_node = new ros::NodeHandle("~");
	_nh.reset(new ros::NodeHandle());
	_nh_priv.reset(new ros::NodeHandle("~"));

	SetupSteeringConstraint();
	SetupEncoders();
}

void CaRINAGazeboPlugin::OnUpdate()
{
	ros::spinOnce();

	OnEncodersUpdate();
}


void CaRINAGazeboPlugin::OnEncodersUpdate() {
	ros::Time t = ros::Time::now();
	ros::Duration d = t - _encoders_last_update;
	if (d.toNSec() > (1000000000.0 / _encoders_update_rate)) {
		_encoders_last_update = t;
		double wheel = _joint_wheel->GetAngle(0).Radian();
		double phi = _joint_steering->GetAngle(0).Radian();
		double enc_wheel = ((wheel - _encoders_acc_wheel) / (2 * M_PI)) * WHEEL_ENCODER_RESOLUTION;
		double enc_steer = (phi / 2 * M_PI) * STEER_ENCODER_RESOLUTION;
		lrm_msgs::Encoders msg;

		_encoders_acc_wheel = wheel;

		//ROS_WARN_STREAM("pos:" << encWheel);

		if (abs(enc_wheel) > WHEEL_ENCODER_RESOLUTION) {
			ROS_ERROR_STREAM("wheel encoder miss" << enc_wheel);
			enc_wheel = 0;
		}

		msg.header.frame_id = "/wheel_encoder";
		msg.header.stamp = t;
		msg.steering.absolute = enc_steer;

		msg.wheel.relative = enc_wheel;
		_encoders_pub.publish(msg);

		//if (steering_wheel_) {
		//	//joint_state_steering_wheel_->commanded_effort_ = 35 * -(encSteer / STEER_ENCODER_RESOLUTION) * (2 * M_PI);
		//}

	}
}
GZ_REGISTER_MODEL_PLUGIN(CaRINAGazeboPlugin);
}
