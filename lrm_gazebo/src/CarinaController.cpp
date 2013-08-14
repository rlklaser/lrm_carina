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
 * @file CarinaController.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 25, 2012
 *
 */

#include <ros/ros.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/robot.h>
#include <geometry_msgs/Twist.h>
#include <pluginlib/class_list_macros.h>

#include <lrm_msgs/Encoders.h>
#include <lrm_description/Constants.h>

//#include <stdlib.h>

namespace lrm_gazebo {

typedef pr2_mechanism_model::RobotState RobotState;
typedef pr2_mechanism_model::JointState JointState;
typedef std::vector<JointState*> JointStateVector;

class CarinaController: public pr2_controller_interface::Controller {

public:
	/*
	 CarinaController()
	 :robot_(0), n_(0)
	 {
	 }
	 */

	bool init(RobotState* robot, ros::NodeHandle& n) {
		robot_ = robot;

		double timeout_s;
		n.param("timeout", timeout_s, 0.200);
		timeout_ = ros::Duration(timeout_s);

		sub_twist_ = n.subscribe("cmd_vel", 10, &CarinaController::callbackTwist, this);
		pub_encoders_ = n.advertise<lrm_msgs::Encoders>("encoders_gazebo", 1);
		lastWheel_ = 0;
		accWheel_ = 0;
		freq_ = 1;

		return loadJoints(n);
	}

	void starting() {
		ROS_WARN_STREAM("Carina controller starting...");

		twist_.linear.x = 0;
		twist_.angular.z = 0;
		last_update_ = robot_->getTime();
		/*
		 odom_.header.frame_id = fixed_frame_;
		 odom_.child_frame_id = base_frame_;
		 */

		// Initial positions traction motors
		init_pos_flw_ = joint_state_flw_->position_;
		init_pos_frw_ = joint_state_frw_->position_;
		init_pos_blw_ = joint_state_blw_->position_;
		init_pos_brw_ = joint_state_brw_->position_;
		init_pos_stw_ = joint_state_stw_->position_;

		// Initial velocities traction motors
		init_vel_flw_ = joint_state_flw_->velocity_;
		init_vel_frw_ = joint_state_frw_->velocity_;
		init_vel_blw_ = joint_state_blw_->velocity_;
		init_vel_brw_ = joint_state_brw_->velocity_;
	}

	void stopping() {
		ROS_WARN_STREAM("Carina controller stopping...");

		joint_state_flw_->commanded_effort_ = 0.0;
		joint_state_frw_->commanded_effort_ = 0.0;
		joint_state_blw_->commanded_effort_ = 0.0;
		joint_state_brw_->commanded_effort_ = 0.0;
	}

	void update() {
		ros::Time t = robot_->getTime();
		// Watchdog: Cancel current twist if it's older than the timeout
		if ((t - last_cmd_) > timeout_) {
			//TODO: uncomment
			//twist_.linear.x = 0;
			//twist_.angular.z = 0;
		}

		// Calculate each wheel's speed according to the current twist
		// command.
		/*
		 double phidot[2] = { 0, 0 };
		 const double& xdot = twist_.linear.x;
		 const double& tdot = twist_.angular.z;
		 phidot[0] = (xdot - l_ * tdot) / r_;
		 phidot[1] = (-xdot - l_ * tdot) / r_;

		 // Update the joint states with the wheels' speeds.
		 // Implements a simple P controller.
		 for (int i = 0; i < 2; ++i)
		 {
		 pr2_mechanism_model::JointState& j = *prop_joints_[i];
		 double err = phidot[i] - j.velocity_;
		 j.commanded_effort_ = p_ * err;
		 }

		 // Update and publish odometry.
		 double dt = (t - last_update_).toSec();
		 double dsl = r_ * prop_joints_[0]->velocity_ * dt;
		 double dsr = -1.0 * r_ * prop_joints_[1]->velocity_ * dt;
		 double ds = (dsl + dsr) / 2.0;
		 double dtheta = (dsr - dsl) / b_;
		 double dx = ds * cos(theta_ + (dtheta / 2.0));
		 double dy = ds * sin(theta_ + (dtheta / 2.0));
		 x_ += dx;
		 y_ += dy;
		 theta_ += dtheta;

		 // Note: no covariance.
		 tf::Quaternion quat = tf::createQuaternionFromYaw(theta_);
		 odom_.header.stamp = t;
		 odom_.pose.pose.position.x = x_;
		 odom_.pose.pose.position.y = y_;
		 tf::quaternionTFToMsg(quat, odom_.pose.pose.orientation);
		 odom_.twist.twist.linear.x = ds / dt;
		 odom_.twist.twist.angular.z = dtheta / dt;
		 pub_odom_.publish(odom_);
		 tf_.sendTransform(tf::StampedTransform(tf::Transform(quat, btVector3(x_, y_, 0)), t, fixed_frame_, base_frame_));
		 */

		double flw_mps, frw_mps, blw_mps, brw_mps;

		// Single wheel linear speeds in m/s
		flw_mps = joint_state_flw_->velocity_ * (DEFAULT_WHEEL_DIAMETER / 2.0);
		frw_mps = joint_state_frw_->velocity_ * (DEFAULT_WHEEL_DIAMETER / 2.0);
		blw_mps = joint_state_blw_->velocity_ * (DEFAULT_WHEEL_DIAMETER / 2.0);
		brw_mps = joint_state_brw_->velocity_ * (DEFAULT_WHEEL_DIAMETER / 2.0);

		// Control - state space control implemented for gazebo simulation
		double kpv = 10.0;
		double kpp = 35.0;

		// Compute state control actions
		// State feedback error 2 position loops / 4 velocity loops
		// Single steering
		double d1 = 0.0;
		double d = DEFAULT_ROBOT_LENGHT;
		double alfa_ref_left = 0.0;
		double alfa_ref_right = 0.0;
		if (twist_.angular.z != 0.0) {
			d1 = d / tan(twist_.angular.z);
			alfa_ref_left = atan2(d, d1 - DEFAULT_ROBOT_WIDTH / 2);
			alfa_ref_right = atan2(d, d1 + DEFAULT_ROBOT_WIDTH / 2);
			if (twist_.angular.z < 0.0) {
				alfa_ref_left = alfa_ref_left - M_PI;
				alfa_ref_right = alfa_ref_right - M_PI;
			}
		} else {
			alfa_ref_left = 0.0;
			alfa_ref_right = 0.0;
		}

		// Single steering
		double up_flw_dir = kpp * (alfa_ref_left - joint_state_flw_dir_->position_);
		double up_frw_dir = kpp * (alfa_ref_right - joint_state_frw_dir_->position_);

		double uv_flw = kpv * (twist_.linear.x - flw_mps);
		double uv_frw = kpv * (twist_.linear.x - frw_mps);
		double uv_blw = kpv * (twist_.linear.x - blw_mps);
		double uv_brw = kpv * (twist_.linear.x - brw_mps);

		// Motor control actions
		//double limit_direction = 10.0;
		//double limit_traction = 20.0;

		// ROS_INFO("Sending control actions alfa_ref=%5.2f  up_flw_dir=%5.2f",alfa_ref_, up_flw_dir );
		joint_state_flw_dir_->commanded_effort_ = up_flw_dir; //saturation(up_flw_dir, -limit_direction, limit_direction);
		joint_state_frw_dir_->commanded_effort_ = up_frw_dir; //saturation(up_frw_dir, -limit_direction, limit_direction);

		//joint_state_stw_->commanded_effort_ = (((up_flw_dir + up_frw_dir) / 2) / 2*M_PI) * WHEEL_ENCODER_RESOLUTION;

		joint_state_flw_->commanded_effort_ = uv_flw; //saturation(uv_flw, -limit_traction, limit_traction);
		joint_state_frw_->commanded_effort_ = uv_frw; //saturation(uv_frw, -limit_traction, limit_traction);
		joint_state_blw_->commanded_effort_ = uv_blw; //saturation(uv_blw, -limit_traction, limit_traction);
		joint_state_brw_->commanded_effort_ = uv_brw; //saturation(uv_brw, -limit_traction, limit_traction);

		joint_state_fls_->commanded_effort_ = 0;
		joint_state_frs_->commanded_effort_ = 0;
		joint_state_stw_->commanded_effort_ = 0;

		//joint_state_fls_->

		last_update_ = t;

		if (freq_ > (1000 / 20) - 1) {
			freq_ = 1;
			//accWheel_ += joint_state_frw_->position_;
			double phi = (joint_state_flw_dir_->position_ + joint_state_frw_dir_->position_) / 2;
			long long encWheel = ((joint_state_frw_->position_ - accWheel_) / (2 * M_PI)) * WHEEL_ENCODER_RESOLUTION;
			long long encSteer = (phi / 2 * M_PI) * STEER_ENCODER_RESOLUTION;
			lrm_msgs::Encoders msg;

			msg.header.frame_id = "/wheel_encoder";
			msg.header.stamp = t; //ros::Time::now();
			msg.steering.absolute = encSteer;

			msg.wheel.relative = encWheel; //- lastWheel_;
			//lastWheel_ = encWheel;
			accWheel_ = joint_state_frw_->position_;
			pub_encoders_.publish(msg);
		} else {
			freq_++;
		}
	}

private:
	RobotState* robot_;
	//ros::NodeHandle& n_;
	//JointStateVector prop_joints_;

	JointState* joint_state_flw_dir_; //front left wheel direction
	JointState* joint_state_frw_dir_; //front right wheel direction
	JointState* joint_state_flw_; //front left wheel
	JointState* joint_state_frw_; //front right wheel
	JointState* joint_state_blw_; //rear left wheel
	JointState* joint_state_brw_; //rear right wheel
	JointState* joint_state_stw_; //steering wheel

	JointState* joint_state_fls_; //front left suspension
	JointState* joint_state_frs_; //front right suspension

	// Joint Positions
	double init_pos_stw_;
	double init_pos_flw_dir_;
	double init_pos_frw_dir_;
	double init_pos_blw_;
	double init_pos_flw_;
	double init_pos_brw_;
	double init_pos_frw_;
	// Joint Speeds
	double init_vel_blw_;
	double init_vel_flw_;
	double init_vel_brw_;
	double init_vel_frw_;

	long long lastWheel_;
	double accWheel_;

	int freq_;

	ros::Subscriber sub_twist_;
	ros::Publisher pub_encoders_;
	geometry_msgs::Twist twist_;
	ros::Time last_cmd_;
	ros::Duration timeout_;
	ros::Time last_update_;

	void callbackTwist(const geometry_msgs::Twist::ConstPtr& msg) {
		// Copy the incoming twist command.
		twist_ = *msg;
		last_cmd_ = robot_->getTime();
	}

	bool loadJoints(ros::NodeHandle& n_) {
		std::string joint_stw;
		if (!n_.getParam("joint_stw", joint_stw)) {
			ROS_ERROR("No joint STW given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_stw_ = robot_->getJointState(joint_stw);

		std::string joint_flw_dir;
		if (!n_.getParam("joint_flw_dir", joint_flw_dir)) {
			ROS_ERROR("No joint FLW DIR given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_flw_dir_ = robot_->getJointState(joint_flw_dir);

		std::string joint_frw_dir;
		if (!n_.getParam("joint_frw_dir", joint_frw_dir)) {
			ROS_ERROR("No joint FRW DIR given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_frw_dir_ = robot_->getJointState(joint_frw_dir);

		std::string joint_blw;
		if (!n_.getParam("joint_blw", joint_blw)) {
			ROS_ERROR("No joint BLW given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_blw_ = robot_->getJointState(joint_blw);

		std::string joint_flw;
		if (!n_.getParam("joint_flw", joint_flw)) {
			ROS_ERROR("No joint FLW given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_flw_ = robot_->getJointState(joint_flw);

		std::string joint_brw;
		if (!n_.getParam("joint_brw", joint_brw)) {
			ROS_ERROR("No joint BRW given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_brw_ = robot_->getJointState(joint_brw);

		std::string joint_frw;
		if (!n_.getParam("joint_frw", joint_frw)) {
			ROS_ERROR("No joint FRW given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_frw_ = robot_->getJointState(joint_frw);

		std::string joint_fls;
		if (!n_.getParam("joint_fls", joint_frw)) {
			ROS_ERROR("No joint FLS given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_fls_ = robot_->getJointState(joint_fls);

		std::string joint_frs;
		if (!n_.getParam("joint_frs", joint_frw)) {
			ROS_ERROR("No joint FRS given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_frs_ = robot_->getJointState(joint_frs);

		return true;
	}
};
}

PLUGINLIB_DECLARE_CLASS(
		lrm_gazebo,
		CarinaControllerPlugin,
		lrm_gazebo::CarinaController,
		pr2_controller_interface::Controller
		)
