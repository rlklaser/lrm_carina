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
 * @file AckermannControllerPlugin.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Dec 13, 2012
 *
 */
#include <algorithm>
#include <assert.h>
#include <boost/math/constants/constants.hpp>

#include "AckermannControllerPlugin.h"

namespace gazebo {

// Constructor
AckermannControllerPlugin::AckermannControllerPlugin() {
	std::string name = "ackermann_controller_plugin_node";
	int argc = 0;
	_use_cmd_vel = false;
	ros::init(argc, NULL, name);
}

// Destructor
AckermannControllerPlugin::~AckermannControllerPlugin() {
	delete rosnode_;
}

// Load the controller
void AckermannControllerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
	parent_ = _parent;
	world_ = _parent->GetWorld();

	ROS_INFO_STREAM("AckermannControllerPlugin parent sensor name: " << parent_->GetName());

	if (!parent_) {
		gzthrow("AckermannControllerPlugin controller requires a Model as its parent");
	}

	this->robotNamespace_ = "";

	if (_sdf->HasElement("robotNamespace")) {
		robotNamespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
	}

	rosnode_ = new ros::NodeHandle(robotNamespace_);

	ROS_INFO("starting ackermann plugin in ns: %s", robotNamespace_.c_str());

	//tf_prefix_ = tf::getPrefixParam(*rosnode_);

	ros::NodeHandle nh_priv("~");
	ros::NodeHandle nh;

	//nh_priv.param<std::string>("tf_prefix", tf_prefix_, "");
	//tf_prefix_ = parent_->GetName() + "/";

	std::string key;
	if (ros::param::search("tf_prefix", key)) {
		ros::param::get(key, tf_prefix_);
	}

	ROS_INFO_STREAM("AckermannPlugin tf_prefix:" << tf_prefix_ << " : " << nh_priv.getNamespace() << " : " << nh.getNamespace());

	if (!_sdf->HasElement("backLeftJoint")) {
		ROS_WARN("Ackermann Drive plugin missing <backLeftJoint>, defaults to backLeftJoint");
		backLeftJointName_ = "backLeftJoint";
	} else {
		backLeftJointName_ = _sdf->GetElement("backLeftJoint")->GetValueString();
	}

	if (!_sdf->HasElement("backRightJoint")) {
		ROS_WARN("Ackermann Drive plugin missing <backRightJoint>, defaults to backRightJoint");
		backRightJointName_ = "backRightJoint";
	} else {
		backRightJointName_ = _sdf->GetElement("backRightJoint")->GetValueString();
	}

	if (!_sdf->HasElement("frontSteerJoint")) {
		ROS_WARN("Ackermann Drive plugin missing <frontSteerJoint>, defaults to frontSteerJoint");
		frontSteerJointName_ = "frontSteerJoint";
	} else {
		frontSteerJointName_ = _sdf->GetElement("frontSteerJoint")->GetValueString();
	}

	/*
	 if (!_sdf->HasElement("steeringWheelJoint")) {
	 ROS_WARN("Ackermann Drive plugin missing <steeringWheelJoint>, defaults to steeringWheelJoint");
	 steeringWheelJointName_ = "steeringWheelJoint";
	 } else {
	 steeringWheelJointName_ = _sdf->GetElement("steeringWheelJoint")->GetValueString();
	 }

	 if (!_sdf->HasElement("wheelEncoderJoint")) {
	 ROS_WARN("Ackermann Drive plugin missing <wheelEncoderJoint>, defaults to wheelEncoderJoint");
	 wheelEncoderJointName_ = "wheelEncoderJoint";
	 } else {
	 wheelEncoderJointName_ = _sdf->GetElement("wheelEncoderJoint")->GetValueString();
	 }
	 */

	if (!_sdf->HasElement("wheelDiameter")) {
		ROS_WARN("Ackermann Drive plugin missing <wheelDiameter>");
		wheelDiameter_ = 0.48;
	} else {
		wheelDiameter_ = _sdf->GetElement("wheelDiameter")->GetValueDouble();
	}

	if (!_sdf->HasElement("driveTorque")) {
		ROS_WARN("Ackermann Drive plugin missing <driveTorque>");
		driveTorque_ = 700000.0;
	} else {
		driveTorque_ = _sdf->GetElement("driveTorque")->GetValueDouble();
	}

	if (!_sdf->HasElement("steerTorque")) {
		ROS_WARN("Ackermann Drive plugin missing <steerTorque>");
		steerTorque_ = 600.0;
	} else {
		steerTorque_ = _sdf->GetElement("steerTorque")->GetValueDouble();
	}

	if (!_sdf->HasElement("maxSteerAngle")) {
		ROS_WARN("Ackermann Drive plugin missing <maxSteerAngle>");
		maxSteerAngle_ = 32.0 * M_PI / 180;
	} else {
		maxSteerAngle_ = _sdf->GetElement("maxSteerAngle")->GetValueDouble() * M_PI / 180;
	}

	if (!_sdf->HasElement("topicName")) {
		ROS_WARN("Ackermann Drive plugin missing <topicName>");
		topicName_ = "cmd_vel";
	} else {
		topicName_ = _sdf->GetElement("topicName")->GetValueString();
	}

	if (!_sdf->HasElement("maxThrottle")) {
		maxThrottle_ = 110;
	} else {
		maxThrottle_ = _sdf->GetElement("maxThrottle")->GetValueDouble();
	}

	/*
	if (!_sdf->HasElement("topicName")) {
		topicName_ = "cmd_vel";
	} else {
		topicName_ = _sdf->GetElement("topicName")->GetValueString();
	}
	*/

	wheelSpeed_[BACKRIGHT] = 0;
	wheelSpeed_[BACKLEFT] = 0;

	joints_[BACKRIGHT] = parent_->GetJoint(/*tf_prefix_ +*/backRightJointName_);
	joints_[BACKLEFT] = parent_->GetJoint(/*tf_prefix_ +*/backLeftJointName_);
	joints_[FRONTSTEER] = parent_->GetJoint(/*tf_prefix_ +*/frontSteerJointName_);
	//joints_[STEERWHEEL] = parent_->GetJoint(steeringWheelJointName_);
	//joints_[WHEELENCODER] = parent_->GetJoint(wheelEncoderJointName_);

	if (!joints_[BACKRIGHT]) {
		gzthrow("The controller couldn't get back right revolute joint");
	}
	if (!joints_[BACKLEFT]) {
		gzthrow("The controller couldn't get back left revolute joint");
	}
	if (!joints_[FRONTSTEER]) {
		gzthrow("The controller couldn't get front steer hinge joint");
	}

	// ROS: Subscribe to the velocity command topic (usually "cmd_vel")
	//ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(tf_prefix_ + topicName_, 1, boost::bind(&AckermannControllerPlugin::CmdVelCallback, this, _1), ros::VoidPtr(), &queue_);
	//sub_ = rosnode_->subscribe(so);

	sub_ = rosnode_->subscribe<geometry_msgs::Twist>(tf_prefix_ + topicName_, 1, &AckermannControllerPlugin::CmdVelCallback, this);
	sub_steer_ = rosnode_->subscribe<lrm_msgs::Steering>(tf_prefix_ + "/steering_commands", 1, &AckermannControllerPlugin::SteeringCallback, this);
	sub_throttle_ = rosnode_->subscribe<lrm_msgs::Throttle>(tf_prefix_ + "/throttle_commands", 1, &AckermannControllerPlugin::ThrottleCallback, this);

	// start custom queue for ackermann drive
	callback_queue_thread_ = boost::thread(boost::bind(&AckermannControllerPlugin::QueueThread, this));

	// listen to the update event (broadcast every simulation iteration)
	connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AckermannControllerPlugin::UpdateChild, this));

	freq_ = 1;
	last_update_ = world_->GetSimTime();
	accWheel_ = 0;

	joints_[BACKLEFT]->SetMaxForce(0, driveTorque_);
	joints_[BACKRIGHT]->SetMaxForce(0, driveTorque_);
	joints_[FRONTSTEER]->SetMaxForce(0, steerTorque_);
}

// Update the controller
void AckermannControllerPlugin::UpdateChild() {
	double stAngle = 0;
	double stepTime = (world_->GetSimTime() - last_update_).Double();
	last_update_ = world_->GetSimTime();

	GetPositionCmd();

	stAngle = joints_[FRONTSTEER]->GetAngle(0).Radian();

	if(_use_cmd_vel) {
		joints_[BACKLEFT]->SetVelocity(0, wheelSpeed_[BACKLEFT] / (wheelDiameter_ / 2.0));
		joints_[BACKRIGHT]->SetVelocity(0, wheelSpeed_[BACKRIGHT] / (wheelDiameter_ / 2.0));
		//joints_[FRONTSTEER]->SetVelocity(0, wheelSpeed_[FRONTSTEER] / 2.0);
		//joints_[FRONTSTEER]->SetAngle(0, wheelSpeed_[FRONTSTEER]);
		//joints_[FRONTSTEER]->SetVelocity(0, (wheelSpeed_[FRONTSTEER] - stAngle) / stepTime);
	} else {
		double l_force = wheelSpeed_[BACKLEFT] * driveTorque_/maxThrottle_;
		double r_force = wheelSpeed_[BACKRIGHT] * driveTorque_/maxThrottle_;

		//l_force /= stepTime;
		//r_force /= stepTime;

		//std::cout << l_force << " ";

		joints_[BACKLEFT]->SetForce(0, l_force);
		joints_[BACKRIGHT]->SetForce(0, r_force);
	}

	double diff = (wheelSpeed_[FRONTSTEER] - stAngle);

	//double sgn = fabs(diff)/diff;

	//1 seconds to turn 30 degrees
	double step = (30 * M_PI / 180) / (1 / stepTime);

	//std::cout << step << " " << stAngle << " " << diff << " " << stepTime << std::endl;

	//only set angle at max steering rate
	if (diff > 0) {
		joints_[FRONTSTEER]->SetAngle(0, stAngle + std::min(step, diff));
	} else if (diff < 0) {
		joints_[FRONTSTEER]->SetAngle(0, stAngle - std::min(step, -diff));
	}

}

// Finalize the controller
void AckermannControllerPlugin::FiniChild() {
	alive_ = false;
	queue_.clear();
	queue_.disable();
	rosnode_->shutdown();
	callback_queue_thread_.join();
}

void AckermannControllerPlugin::GetPositionCmd() {
	lock_.lock();

	double vr, va;

	vr = x_;
	if (rot_ != 0) {
		double sgn_rot = fabs(rot_) / rot_;
		va = sgn_rot * std::min(fabs(rot_), maxSteerAngle_);
	} else {
		va = 0;
	}

	wheelSpeed_[BACKLEFT] = vr;
	wheelSpeed_[BACKRIGHT] = vr;
	wheelSpeed_[FRONTSTEER] = va;

	//std::cout << "callback:" << vr << " : " << va << std::endl;

	lock_.unlock();
}

void AckermannControllerPlugin::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
	lock_.lock();

	x_ = cmd_msg->linear.x;
	rot_ = cmd_msg->angular.z;

	_use_cmd_vel = true;

	lock_.unlock();
}

void AckermannControllerPlugin::SteeringCallback(const lrm_msgs::Steering::ConstPtr& cmd_msg) {
	lock_.lock();

	rot_ = angles::from_degrees(cmd_msg->angle);

	lock_.unlock();
}

void AckermannControllerPlugin::ThrottleCallback(const lrm_msgs::Throttle::ConstPtr& cmd_msg) {
	lock_.lock();

	x_ = cmd_msg->value;

	if(_use_cmd_vel) {
		ROS_ERROR_STREAM("Using cmd_vel with cruise_control");
	}

	lock_.unlock();
}

void AckermannControllerPlugin::QueueThread() {
	static const double timeout = 0.01;

	while (alive_ && rosnode_->ok()) {
		queue_.callAvailable(ros::WallDuration(timeout));
	}
}

GZ_REGISTER_MODEL_PLUGIN(AckermannControllerPlugin)
}

