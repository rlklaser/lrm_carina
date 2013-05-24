/*
 * Copyright (c) 2012, Siddhant Ahuja (Sid)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Siddhant Ahuja (Sid) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Desc: Gazebo 1.x plugin for a Ackermann Drive Simulation Robot
 * Adapted from the Erratic and Turtlebot Robot plugin
 * Author: Siddhant Ahuja
 */

#include <algorithm>
#include <assert.h>

#include <boost/bind.hpp>

#include "simBotBaseAckermannController.h"
#include <lrm_msgs/RobotStates.h>

#include <common/common.h>
#include <math/gzmath.h>
#include <physics/physics.h>
#include <sdf/sdf.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/math/constants/constants.hpp>

namespace gazebo {

enum {
	BACKRIGHT, BACKLEFT, FRONTSTEER
};

// Constructor
AckermannPlugin::AckermannPlugin() {
}

// Destructor
AckermannPlugin::~AckermannPlugin() {
	delete rosnode_;
	delete transform_broadcaster_;
}

// Load the controller
void AckermannPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
	this->parent = _parent;
	this->world = _parent->GetWorld();

	gzdbg<< "plugin parent sensor name: " << parent->GetName() << "\n";

	if (!this->parent) {
		gzthrow("Ackermann_Position2d controller requires a Model as its parent");
	}

	this->robotNamespace = "";

	if (_sdf->HasElement("robotNamespace")) {
		this->robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
	}

	if (!_sdf->HasElement("backLeftJoint")) {
		ROS_WARN("Ackermann Drive plugin missing <backLeftJoint>, defaults to backLeftJoint");
		this->backLeftJointName = "backLeftJoint";
	} else {
		this->backLeftJointName = _sdf->GetElement("backLeftJoint")->GetValueString();
	}

	if (!_sdf->HasElement("backRightJoint")) {
		ROS_WARN("Ackermann Drive plugin missing <backRightJoint>, defaults to backRightJoint");
		this->backRightJointName = "backRightJoint";
	} else {
		this->backRightJointName = _sdf->GetElement("backRightJoint")->GetValueString();
	}

	if (!_sdf->HasElement("frontSteerJoint")) {
		ROS_WARN("Ackermann Drive plugin missing <frontSteerJoint>, defaults to frontSteerJoint");
		this->frontSteerJointName = "frontSteerJoint";
	} else {
		this->frontSteerJointName = _sdf->GetElement("frontSteerJoint")->GetValueString();
	}
/*
	if (!_sdf->HasElement("wheelTrack")) {
		ROS_WARN("Ackermann Drive plugin missing <wheelTrack>, defaults to 0.34");
		this->wheelTrack = 0.34;
	} else {
		this->wheelTrack = _sdf->GetElement("wheelTrack")->GetValueDouble();
	}

	if (!_sdf->HasElement("wheelBase")) {
		ROS_WARN("Ackermann Drive plugin missing <wheelBase>, defaults to 0.34");
		this->wheelBase = 0.34;
	} else {
		this->wheelBase = _sdf->GetElement("wheelBase")->GetValueDouble();
	}
*/
	if (!_sdf->HasElement("wheelDiameter")) {
		ROS_WARN("Ackermann Drive plugin missing <wheelDiameter>, defaults to 0.15");
		this->wheelDiameter = 0.15;
	} else {
		this->wheelDiameter = _sdf->GetElement("wheelDiameter")->GetValueDouble();
	}

	if (!_sdf->HasElement("driveTorque")) {
		ROS_WARN("Ackermann Drive plugin missing <driveTorque>, defaults to 5.0");
		this->driveTorque = 5.0;
	} else {
		this->driveTorque = _sdf->GetElement("driveTorque")->GetValueDouble();
	}

	if (!_sdf->HasElement("steerTorque")) {
		ROS_WARN("Ackermann Drive plugin missing <steerTorque>, defaults to 5.0");
		this->steerTorque = 5.0;
	} else {
		this->steerTorque = _sdf->GetElement("steerTorque")->GetValueDouble();
	}

	if (!_sdf->HasElement("maxSteerAngle")) {
		ROS_WARN("Ackermann Drive plugin missing <maxSteerAngle>, defaults to 20.0");
		this->maxSteerAngle = 20.0;
	} else {
		this->maxSteerAngle = _sdf->GetElement("maxSteerAngle")->GetValueDouble();
	}

	// Velocity Control on Front Steer Joint
	//	true : Velocity Control
	//	false : Direct steer angle control
	/*if (!_sdf->HasElement("velControl"))
	 {
	 ROS_WARN("Ackermann Drive plugin missing <velControl>, defaults to true");
	 this->velControl = true;
	 }
	 else
	 {
	 this->velControl = _sdf->GetElement("velControl")->GetValueBool();
	 }

	 */
	if (!_sdf->HasElement("topicName")) {
		ROS_WARN("Ackermann Drive plugin missing <topicName>, defaults to cmd_vel");
		this->topicName = "cmd_vel";
	} else {
		this->topicName = _sdf->GetElement("topicName")->GetValueString();
	}

	wheelSpeed[BACKRIGHT] = 0;
	wheelSpeed[BACKLEFT] = 0;
	steerAngle = 0;
	turnAngle = 0;
	x_ = 0;
	rot_ = 0;
	alive_ = true;
	sigma_x_ = 0.002;
	sigma_y_ = 0.002;
	sigma_theta_ = 0.017;
	cov_x_y_ = 0.0;
	cov_x_theta_ = 0.0;
	cov_y_theta_ = 0.0;

	joints[BACKRIGHT] = this->parent->GetJoint(backRightJointName);
	joints[BACKLEFT] = this->parent->GetJoint(backLeftJointName);
	joints[FRONTSTEER] = this->parent->GetJoint(frontSteerJointName);

	if (!joints[BACKRIGHT]) {
		gzthrow("The controller couldn't get back right revolute joint");
	}
	if (!joints[BACKLEFT]) {
		gzthrow("The controller couldn't get back left revolute joint");
	}
	if (!joints[FRONTSTEER]) {
		gzthrow("The controller couldn't get front steer hinge joint");
	}

	// Initialize the ROS node and subscribe to cmd_vel
	int argc = 0;
	char** argv = NULL;
	ros::init(argc, argv, "simBotBaseAckermannController", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
	rosnode_ = new ros::NodeHandle(this->robotNamespace);

	ROS_INFO("starting ackermann plugin in ns: %s", this->robotNamespace.c_str());

	tf_prefix_ = tf::getPrefixParam(*rosnode_);
	transform_broadcaster_ = new tf::TransformBroadcaster();

	// ROS: Subscribe to the velocity command topic (usually "cmd_vel")
	ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(topicName, 1, boost::bind(&AckermannPlugin::cmdVelCallback, this, _1), ros::VoidPtr(), &queue_);
	sub_ = rosnode_->subscribe(so);
	pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);
	pub_states_ = rosnode_->advertise<lrm_msgs::RobotStates>("robot_states", 1);

	// Initialize the controller
	// Reset odometric pose
	wheelOdomPose[0] = 0.0;
	wheelOdomPose[1] = 0.0;
	wheelOdomPose[2] = 0.0;

	wheelOdomVel[0] = 0.0;
	wheelOdomVel[1] = 0.0;
	wheelOdomVel[2] = 0.0;

	// start custom queue for ackermann drive
	this->callback_queue_thread_ = boost::thread(boost::bind(&AckermannPlugin::QueueThread, this));

	// listen to the update event (broadcast every simulation iteration)
	this->updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&AckermannPlugin::UpdateChild, this));
}

// Update the controller
void AckermannPlugin::UpdateChild() {
	// TODO: Step should be in a parameter of this function
	double wDia, wBase;//, wTrack;
	double d1, d2, stAngle;
	double dr, da, r;
	double stepTime = this->world->GetPhysicsEngine()->GetStepTime();

	GetPositionCmd();

	wDia = wheelDiameter;
	//wTrack = wheelTrack;

	// Distance travelled by back wheels
	d1 = stepTime * wDia / 2.0 * joints[BACKLEFT]->GetVelocity(0);
	d2 = stepTime * wDia / 2.0 * joints[BACKRIGHT]->GetVelocity(0);

	// Steering Angle 
	stAngle = joints[FRONTSTEER]->GetAngle(0).GetAsRadian();
	//ROS_INFO("Current stAngle in Radians=%2.4f", stAngle);

	// Find Radius of Curvature
	r = wBase * tan((2 * boost::math::constants::pi<double>()) / 4.0 - stAngle);
	//ROS_INFO("Radius of Curvature=%2.4f", r);

	// Compute Displacement
	dr = (d1 + d2) / 2;
	da = dr / r;

	//ROS_INFO("Displacement Angle=%2.4f", da);

	// Compute odometric pose
	wheelOdomPose[0] += dr * cos(wheelOdomPose[2]);
	wheelOdomPose[1] += dr * sin(wheelOdomPose[2]);
	wheelOdomPose[2] += da;

	// Compute odometric instantaneous velocity
	wheelOdomVel[0] = dr / stepTime;
	wheelOdomVel[1] = 0.0;
	wheelOdomVel[2] = da / stepTime;

	joints[BACKLEFT]->SetVelocity(0, wheelSpeed[BACKLEFT] / (wheelDiameter / 2.0));
	joints[BACKRIGHT]->SetVelocity(0, wheelSpeed[BACKRIGHT] / (wheelDiameter / 2.0));

	joints[FRONTSTEER]->SetVelocity(0, wheelSpeed[FRONTSTEER] / 2.0);

	/*if(velControl)
	 {
	 joints[FRONTSTEER]->SetVelocity(0, wheelSpeed[FRONTSTEER] / 2.0);
	 }
	 else
	 {
	 //Changes by: Jimmy Yu, University of Waterloo
	 diffAngle = wheelSpeed[FRONTSTEER] * maxSteerAngle - turnAngle; // get diff between current and goal, assumes max turning steering angle
	 if ( std::abs( diffAngle ) > 0.1 ) // check against threshold
	 joints[FRONTSTEER]->SetVelocity( 0 , diffAngle > 0 ? 89 : -89 ); // set to max velocity possible in necessary direction
	 turnAngle = joints[FRONTSTEER]->GetAngle( 0 ).GetAsDegree(); // save current angle
	 ROS_INFO("Turn Angle=%2.4f", turnAngle);
	 std::cout << turnAngle << std::endl; // output for debugging
	 }*/

	//ROS_INFO("Applied Angular Velocity=%2.4f",  wheelSpeed[FRONTSTEER] / 2.0);
	joints[BACKLEFT]->SetMaxForce(0, driveTorque);
	joints[BACKRIGHT]->SetMaxForce(0, driveTorque);
	joints[FRONTSTEER]->SetMaxForce(0, steerTorque);

	//write_position_data();
	publish_wheel_odometry(stAngle);
}

// Finalize the controller
void AckermannPlugin::FiniChild() {
	alive_ = false;
	queue_.clear();
	queue_.disable();
	rosnode_->shutdown();
	callback_queue_thread_.join();
}

void AckermannPlugin::GetPositionCmd() {
	lock.lock();

	double vr, va;

	vr = x_; //myIface->data->cmdVelocity.pos.x;
	va = rot_; //myIface->data->cmdVelocity.yaw;

	//std::cout << "X: [" << x_ << "] ROT: [" << rot_ << "]" << std::endl;

	wheelSpeed[BACKLEFT] = vr;
	wheelSpeed[BACKRIGHT] = vr;
	wheelSpeed[FRONTSTEER] = va;

	lock.unlock();
}

void AckermannPlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
	lock.lock();

	x_ = cmd_msg->linear.x;
	rot_ = cmd_msg->angular.z;

	lock.unlock();
}

void AckermannPlugin::QueueThread() {
	static const double timeout = 0.01;

	while (alive_ && rosnode_->ok()) {
		queue_.callAvailable(ros::WallDuration(timeout));
	}
}

void AckermannPlugin::populateCovariance(nav_msgs::Odometry &msg) {
	if (fabs(msg.twist.twist.linear.x) <= 1e-8 && fabs(msg.twist.twist.linear.y) <= 1e-8 && fabs(msg.twist.twist.angular.z) <= 1e-8) {
		//nav_msgs::Odometry has a 6x6 covariance matrix
		msg.pose.covariance[0] = 1e-12;
		msg.pose.covariance[7] = 1e-12;
		msg.pose.covariance[35] = 1e-12;

		msg.pose.covariance[1] = 1e-12;
		msg.pose.covariance[6] = 1e-12;

		msg.pose.covariance[31] = 1e-12;
		msg.pose.covariance[11] = 1e-12;

		msg.pose.covariance[30] = 1e-12;
		msg.pose.covariance[5] = 1e-12;
	} else {
		// nav_msgs::Odometry has a 6x6 covariance matrix
		msg.pose.covariance[0] = pow(sigma_x_, 2);
		msg.pose.covariance[7] = pow(sigma_y_, 2);
		msg.pose.covariance[35] = pow(sigma_theta_, 2);

		msg.pose.covariance[1] = cov_x_y_;
		msg.pose.covariance[6] = cov_x_y_;

		msg.pose.covariance[31] = cov_y_theta_;
		msg.pose.covariance[11] = cov_y_theta_;

		msg.pose.covariance[30] = cov_x_theta_;
		msg.pose.covariance[5] = cov_x_theta_;

	}

	msg.pose.covariance[14] = DBL_MAX;
	msg.pose.covariance[21] = DBL_MAX;
	msg.pose.covariance[28] = DBL_MAX;

	msg.twist.covariance = msg.pose.covariance;
}

void AckermannPlugin::publish_wheel_odometry(double stAngle) {
	ros::Time current_time = ros::Time::now();
	std::string odom_frame = tf::resolve(tf_prefix_, "odom");
	std::string base_footprint_frame = tf::resolve(tf_prefix_, "base_footprint");

	// getting data for base_footprint to odom transform
	math::Pose pose = this->parent->GetState().GetPose();

	btQuaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
	btVector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

	tf::Transform base_footprint_to_odom(qt, vt);
	transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame, base_footprint_frame));

	// publish odom topic
	wheel_odom_.pose.pose.position.x = pose.pos.x;
	wheel_odom_.pose.pose.position.y = pose.pos.y;

	wheel_odom_.pose.pose.orientation.x = pose.rot.x;
	wheel_odom_.pose.pose.orientation.y = pose.rot.y;
	wheel_odom_.pose.pose.orientation.z = pose.rot.z;
	wheel_odom_.pose.pose.orientation.w = pose.rot.w;

	math::Vector3 linear = this->parent->GetWorldLinearVel();
	wheel_odom_.twist.twist.linear.x = linear.x;
	wheel_odom_.twist.twist.linear.y = linear.y;
	wheel_odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;

	wheel_odom_.header.stamp = current_time;
	wheel_odom_.header.frame_id = odom_frame;
	wheel_odom_.child_frame_id = base_footprint_frame;

	populateCovariance(wheel_odom_);

	pub_.publish(wheel_odom_);

	//Publish Robot States message

	msg_states_.header.stamp = current_time;
	msg_states_.x = pose.pos.x;
	msg_states_.y = pose.pos.y;
	msg_states_.z = pose.pos.z;
	msg_states_.x_dot = linear.x;
	msg_states_.y_dot = linear.y;
	msg_states_.z_dot = linear.z;

	double roll, pitch, yaw;

	btMatrix3x3(qt).getEulerYPR(yaw, pitch, roll);
	msg_states_.roll_phi = roll;
	msg_states_.pitch_theta = pitch;
	msg_states_.yaw_psi = yaw;

	math::Vector3 angular = this->parent->GetWorldAngularVel();
	msg_states_.p_rate = angular.y;
	msg_states_.q_rate = angular.x;
	msg_states_.r_rate = angular.z;

	msg_states_.steerAngle = stAngle;

	pub_states_.publish(msg_states_);

}

// Update the data in the interface
void AckermannPlugin::write_position_data() {
	// // TODO: Data timestamp
	// pos_iface_->data->head.time = Simulator::Instance()->GetSimTime().Double();

	// pose.pos.x = odomPose[0];
	// pose.pos.y = odomPose[1];
	// pose.rot.GetYaw() = NORMALIZE(odomPose[2]);

	// pos_iface_->data->velocity.pos.x = odomVel[0];
	// pos_iface_->data->velocity.yaw = odomVel[2];

	math::Pose orig_pose = this->parent->GetWorldPose();

	math::Pose new_pose = orig_pose;
	new_pose.pos.x = wheelOdomPose[0];
	new_pose.pos.y = wheelOdomPose[1];
	new_pose.rot.SetFromEuler(math::Vector3(0, 0, wheelOdomPose[2]));

	this->parent->SetWorldPose(new_pose);
}

GZ_REGISTER_MODEL_PLUGIN(AckermannPlugin)
}
