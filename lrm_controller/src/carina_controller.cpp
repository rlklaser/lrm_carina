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
 * @file carina_controller.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 25, 2012
 *
 */

#include <ros/ros.h>

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/robot.h>

#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

#include <geometry_msgs/Twist.h>
#include <lrm_description/Constants.h>

namespace lrm_controller {

typedef pr2_mechanism_model::RobotState RobotState;
typedef pr2_mechanism_model::JointState JointState;
typedef std::vector<JointState*> JointStateVector;

class CarinaController: public pr2_controller_interface::Controller {
public:

	bool init(RobotState* robot, ros::NodeHandle& n) {
		robot_ = robot;
		//ROS_WARN_STREAM("CaRINA controller loading joints...");

		loadJoints(n);

		sub_twist_ = n.subscribe("cmd_vel", 10, &CarinaController::callbackTwist, this);

		return true;
	}

	void starting() {
		ROS_WARN_STREAM("CaRINA controller starting...");
	}

	void stopping() {
		ROS_WARN_STREAM("CaRINA controller stopping...");
	}

	void update() {
		//ROS_WARN_STREAM("CaRINA controller updating...");

		ros::Time t = robot_->getTime();

		// Watchdog: Cancel current twist if it's older than the timeout
		//if ((t - last_cmd_) > timeout_) {
		//	twist_.linear.x = 0;
		//	twist_.angular.z = 0;
		//}

		double blw_mps, brw_mps;

		// Single wheel linear speeds in m/s
		blw_mps = joint_state_brw_->velocity_ * (wheel_diameter_ / 2.0);
		brw_mps = joint_state_brw_->velocity_ * (wheel_diameter_ / 2.0);

		// Control - state space control implemented for gazebo simulation
		double kpv = 10.0;
		double kpp = 30.0;

		double up_frw_dir = kpp * (twist_.angular.z - joint_state_stw_->position_);
		double uv_blw = kpv * (twist_.linear.x - blw_mps);
		double uv_brw = kpv * (twist_.linear.x - brw_mps);

		//ROS_WARN_STREAM("CaRINA controller updating2...");

		joint_state_stw_->commanded_effort_ = up_frw_dir; //saturation(up_frw_dir, -limit_direction, limit_direction);
		joint_state_blw_->commanded_effort_ = uv_blw; //saturation(uv_blw, -limit_traction, limit_traction);
		joint_state_brw_->commanded_effort_ = uv_brw; //saturation(uv_brw, -limit_traction, limit_traction);

		//ROS_WARN_STREAM("CaRINA controller updated");

	}

private:

	RobotState* robot_;
	JointState* joint_state_;
	double init_pos_;
	double effort_;
	geometry_msgs::Twist twist_;
	ros::Time last_cmd_;
	ros::Subscriber sub_twist_;
	ros::Duration timeout_;

	JointState* joint_state_blw_; //rear left wheel
	JointState* joint_state_brw_; //rear right wheel
	JointState* joint_state_stw_; //steering

	double wheel_diameter_;
	std::string tf_prefix_;

	//HACK!
	double steer_start_angle_;

	bool loadJoints(ros::NodeHandle& n_) {

		std::string joint;

		n_.param<std::string>("tf_prefix", tf_prefix_, "");
		ROS_WARN_STREAM("CarinaController tf_prefix:" << tf_prefix_ << " ns:" << n_.getNamespace());

		std::string joint_stw;
		if (!n_.getParam("joint_stw", joint_stw)) {
			ROS_ERROR("No joint [joint_stw] given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_stw_ = robot_->getJointState(joint_stw);

		steer_start_angle_ = joint_state_stw_->position_;
		//ROS_WARN_STREAM(">>>>>dir pos:" << joint_state_stw_->position_);

		std::string joint_brw;
		if (!n_.getParam("joint_brw", joint_brw)) {
			ROS_ERROR("No joint [joint_brw] given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_brw_ = robot_->getJointState(joint_brw);

		std::string joint_blw;
		if (!n_.getParam("joint_blw", joint_blw)) {
			ROS_ERROR("No joint [joint_blw] given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_blw_ = robot_->getJointState(joint_blw);

		double timeout_s;
		n_.param<double>("timeout", timeout_s, 0.200);
		timeout_ = ros::Duration(timeout_s);

		n_.param<double>("wheel_diameter", wheel_diameter_, DEFAULT_WHEEL_DIAMETER);

		//ROS_WARN_STREAM("CaRINA joints ok!");

		return true;
	}

	void callbackTwist(const geometry_msgs::Twist::ConstPtr& msg) {
		// Copy the incoming twist command.
		twist_ = *msg;
		last_cmd_ = robot_->getTime();
	}
};
}

PLUGINLIB_EXPORT_CLASS(lrm_controller::CarinaController, pr2_controller_interface::Controller)
