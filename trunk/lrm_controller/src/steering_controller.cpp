/*
 * carina_controller
 * Copyright (c) 2011, Robotnik Automation, SLL
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
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
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
 *
 * \author Roberto Guzman (rguzman@robotnik.es)
 * \brief Control direction axes pair 2/4 and traction motors 4W for the carina Ackerman single/dual direction drive kinematics
 associate direction/traction wheel joints with motors, apply the control correction in closed loop for the motors and set the commands received by the
 joystick/keyboard teleop node.
 */

#include "lrm_controller/steering_controller.h"
#include <pluginlib/class_list_macros.h>

namespace lrm_controller
{

bool SteeringController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{

	n.param<double>("wheel_diameter", wheel_diameter, DEFAULT_WHEEL_DIAMETER);
	n.param<double>("robot_lenght", robot_lenght, DEFAULT_ROBOT_LENGHT);
	n.param<double>("robot_width", robot_width, DEFAULT_ROBOT_WIDTH);
	n.param<std::string>("tf_prefix", tf_prefix, "");

	ROS_WARN_STREAM("Steering Controller tf_prefix:" << tf_prefix << " ns:" << n.getNamespace());

	std::string joint_drw;
	if (!n.getParam("joint_drw", joint_drw))
	{
		ROS_ERROR("No joint DRW given in namespace: '%s')", n.getNamespace().c_str());
		return false;
	}
	joint_state_drw_ = robot->getJointState(joint_drw);
	if (!joint_state_drw_)
	{
		ROS_ERROR("CarinaController could not find joint named '%s'", joint_drw.c_str());
		return false;
	}
	ROS_DEBUG("CarinaController found joint named '%s'", joint_drw.c_str());

	// front left wheel direction
	std::string joint_flw_dir;
	if (!n.getParam("joint_flw_dir", joint_flw_dir))
	{
		ROS_ERROR("No joint FLW DIR given in namespace: '%s')", n.getNamespace().c_str());
		return false;
	}
	joint_state_flw_dir_ = robot->getJointState(joint_flw_dir);
	if (!joint_state_flw_dir_)
	{
		ROS_ERROR("CarinaController could not find joint named '%s'", joint_flw_dir.c_str());
		return false;
	}
	ROS_DEBUG("CarinaController found joint named '%s'", joint_flw_dir.c_str());

	std::string joint_frw_dir;
	if (!n.getParam("joint_frw_dir", joint_frw_dir))
	{
		ROS_ERROR("No joint FRW DIR given in namespace: '%s')", n.getNamespace().c_str());
		return false;
	}
	joint_state_frw_dir_ = robot->getJointState(joint_frw_dir);
	if (!joint_state_frw_dir_)
	{
		ROS_ERROR("CarinaController could not find joint named '%s'", joint_frw_dir.c_str());
		return false;
	}
	ROS_DEBUG("CarinaController found joint named '%s'", joint_frw_dir.c_str());

	/////////////////////////////////////////
	// Traction Axes
	// back left wheel
	std::string joint_blw;
	if (!n.getParam("joint_blw", joint_blw))
	{
		ROS_ERROR("No joint BLW given in namespace: '%s')", n.getNamespace().c_str());
		return false;
	}
	joint_state_blw_ = robot->getJointState(joint_blw);
	if (!joint_state_blw_)
	{
		ROS_ERROR("CarinaController could not find joint named '%s'", joint_blw.c_str());
		return false;
	}
	ROS_DEBUG("CarinaController found joint named '%s'", joint_blw.c_str());

	// front left wheel
	std::string joint_flw;
	if (!n.getParam("joint_flw", joint_flw))
	{
		ROS_ERROR("No joint FLW given in namespace: '%s')", n.getNamespace().c_str());
		return false;
	}
	joint_state_flw_ = robot->getJointState(joint_flw);
	if (!joint_state_flw_)
	{
		ROS_ERROR("CarinaController could not find joint named '%s'", joint_flw.c_str());
		return false;
	}
	ROS_DEBUG("CarinaController found joint named '%s'", joint_flw.c_str());

	// back right wheel
	std::string joint_brw;
	if (!n.getParam("joint_brw", joint_brw))
	{
		ROS_ERROR("No joint BRW given in namespace: '%s')", n.getNamespace().c_str());
		return false;
	}
	joint_state_brw_ = robot->getJointState(joint_brw);
	if (!joint_state_brw_)
	{
		ROS_ERROR("CarinaController could not find joint named '%s'", joint_brw.c_str());
		return false;
	}
	ROS_DEBUG("CarinaController found joint named '%s'", joint_brw.c_str());

	// front right wheel
	std::string joint_frw;
	if (!n.getParam("joint_frw", joint_frw))
	{
		ROS_ERROR("No joint FRW given in namespace: '%s')", n.getNamespace().c_str());
		return false;
	}
	joint_state_frw_ = robot->getJointState(joint_frw);
	if (!joint_state_frw_)
	{
		ROS_ERROR("CarinaController could not find joint named '%s'", joint_frw.c_str());
		return false;
	}

	// Robot Speeds
	linearSpeedMps_ = 0.0;
	alfaRad_ = 0.0;

	// Robot Positions
	robot_pose_px_ = 0.0;
	robot_pose_py_ = 0.0;
	robot_pose_pa_ = 0.0;

	// Robot state space control references
	v_ref_ = 0.0;
	alfa_ref_ = 0.0;

	// Frequency
	frequency = 100.0; // Hz 50.0
	node_ = n;

	cmd_sub_ = node_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &SteeringController::commandCallback, this);

	return true;
}

/// Controller startup in realtime
void SteeringController::starting()
{
	ROS_WARN_STREAM("CaRINA steering controller starting...");

	init_pos_drw_ = joint_state_drw_->position_;

	// Initial positions traction motors
	init_pos_flw_ = joint_state_flw_->position_;
	init_pos_frw_ = joint_state_frw_->position_;
	init_pos_blw_ = joint_state_blw_->position_;
	init_pos_brw_ = joint_state_brw_->position_;

	// Initial velocities traction motors
	init_vel_flw_ = joint_state_flw_->velocity_;
	init_vel_frw_ = joint_state_frw_->velocity_;
	init_vel_blw_ = joint_state_blw_->velocity_;
	init_vel_brw_ = joint_state_brw_->velocity_;
}

/// Controller update loop in realtime
void SteeringController::update()
{

	double flw_mps, frw_mps, blw_mps, brw_mps;

	// Single wheel linear speeds in m/s
	flw_mps = joint_state_flw_->velocity_ * (wheel_diameter / 2.0);
	frw_mps = joint_state_frw_->velocity_ * (wheel_diameter / 2.0);
	blw_mps = joint_state_blw_->velocity_ * (wheel_diameter / 2.0);
	brw_mps = joint_state_brw_->velocity_ * (wheel_diameter / 2.0);

	// Control - state space control implemented for gazebo simulation
	double kpv = 10.0;
	double kpp = 35.0;

	// Compute state control actions
	// State feedback error 4 position loops / 4 velocity loops
	// Single steering
	double d1 = 0.0;
	double d = robot_lenght; // divide by 2 for dual steering
	double alfa_ref_left = 0.0;
	double alfa_ref_right = 0.0;
	if (alfa_ref_ != 0.0)
	{  // div/0
		d1 = d / tan(alfa_ref_);
		alfa_ref_left = atan2(d, d1 - robot_width / 2);
		alfa_ref_right = atan2(d, d1 + robot_width / 2);
		if (alfa_ref_ < 0.0)
		{
			alfa_ref_left = alfa_ref_left - M_PI;
			alfa_ref_right = alfa_ref_right - M_PI;
		}
	}
	else
	{
		alfa_ref_left = 0.0;
		alfa_ref_right = 0.0;
	}

	// Single steering
	double up_flw_dir = kpp * (alfa_ref_left - joint_state_flw_dir_->position_);
	double up_frw_dir = kpp * (alfa_ref_right - joint_state_frw_dir_->position_);

	double uv_flw = kpv * (v_ref_ - flw_mps);
	double uv_frw = kpv * (v_ref_ - frw_mps);
	double uv_blw = kpv * (v_ref_ - blw_mps);
	double uv_brw = kpv * (v_ref_ - brw_mps);

	// Motor control actions
	double limit_direction = 10.0;
	double limit_traction = 20.0;

	// ROS_INFO("Sending control actions alfa_ref=%5.2f  up_flw_dir=%5.2f",alfa_ref_, up_flw_dir );
	double l_steer = saturation(up_flw_dir, -limit_direction, limit_direction);
	double r_steer = saturation(up_frw_dir, -limit_direction, limit_direction);

	joint_state_flw_dir_->commanded_effort_ = l_steer;
	joint_state_frw_dir_->commanded_effort_ = r_steer;
	joint_state_drw_->commanded_effort_ = (r_steer + l_steer) / 2;

	joint_state_flw_->commanded_effort_ = saturation(uv_flw, -limit_traction, limit_traction);
	joint_state_frw_->commanded_effort_ = saturation(uv_frw, -limit_traction, limit_traction);
	joint_state_blw_->commanded_effort_ = saturation(uv_blw, -limit_traction, limit_traction);
	joint_state_brw_->commanded_effort_ = saturation(uv_brw, -limit_traction, limit_traction);

}

/// Controller stopping in realtime
void SteeringController::stopping()
{
	joint_state_flw_->commanded_effort_ = 0.0;
	joint_state_frw_->commanded_effort_ = 0.0;
	joint_state_blw_->commanded_effort_ = 0.0;
	joint_state_brw_->commanded_effort_ = 0.0;
}

// Set the base velocity command
void SteeringController::setCommand(const geometry_msgs::Twist &cmd_vel)
{
	v_ref_ = saturation(cmd_vel.linear.x, -10.0, 10.0);
	alfa_ref_ = saturation(cmd_vel.angular.z, -0.52, 0.52); // should be close to urdf / robot limits
}

void SteeringController::commandCallback(const geometry_msgs::TwistConstPtr& msg)
{
	base_vel_msg_ = *msg;
	this->setCommand(base_vel_msg_);
}

double SteeringController::saturation(double u, double min, double max)
{
	if (u > max)
		u = max;
	if (u < min)
		u = min;
	return u;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(lrm_controller::SteeringController, pr2_controller_interface::Controller)
