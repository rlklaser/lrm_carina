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
 * @file EncodersController.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Dec 18, 2012
 *
 */

#include <ros/ros.h>

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/robot.h>

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

#include <lrm_msgs/Encoders.h>

#include <lrm_description/Constants.h>

namespace lrm_gazebo {

typedef pr2_mechanism_model::RobotState RobotState;
typedef pr2_mechanism_model::JointState JointState;
typedef std::vector<JointState*> JointStateVector;

class EncodersController: public pr2_controller_interface::Controller {
public:

	bool init(RobotState* robot, ros::NodeHandle& n) {
		robot_ = robot;
		last_update_ = robot_->getTime();
		accWheel_ = 0;
		pub_encoders_ = n.advertise<lrm_msgs::Encoders>("encoders", 2);
		return loadJoints(n);
	}

	void starting() {
		ROS_WARN_STREAM("Encoders controller starting...");
	}

	void stopping() {
		ROS_WARN_STREAM("Encoders controller stopping...");
	}

	void update() {
		ros::Time t = robot_->getTime();
		ros::Duration d = t - last_update_;
		if (d.toNSec() > (1000000000.0 / update_rate_)) {
			//ROS_WARN_STREAM("ping...");
			last_update_ = t;
			double wheel = joint_state_wheel_encoder_->position_;
			double phi = joint_state_steering_encoder_->position_;
			double encWheel = ((wheel - accWheel_) / (2 * M_PI)) * WHEEL_ENCODER_RESOLUTION;
			double encSteer = (phi / 2 * M_PI) * STEER_ENCODER_RESOLUTION;
			lrm_msgs::Encoders msg;

			accWheel_ = wheel;
			//ROS_WARN_STREAM("pos:" << wheel);

			msg.header.frame_id = "/wheel_encoder";
			msg.header.stamp = t;
			msg.steering.absolute = encSteer;

			msg.wheel.relative = encWheel;
			pub_encoders_.publish(msg);

			if (steering_wheel_) {
				//joint_state_steering_wheel_->commanded_effort_ = 35 * -(encSteer / STEER_ENCODER_RESOLUTION) * (2 * M_PI);
			}

		}
	}

private:

	ros::Time last_update_;
	ros::Publisher pub_encoders_;
	RobotState* robot_;
	JointState* joint_state_wheel_encoder_;
	JointState* joint_state_steering_encoder_;
	JointState* joint_state_steering_wheel_;
	double update_rate_;
	bool steering_wheel_;
	double accWheel_;

	bool loadJoints(ros::NodeHandle& n_) {

		std::string tf_prefix_;

		//n_.param("steering_wheel", steering_wheel_, false);
		n_.param("update_rate", update_rate_, 10.0);
		n_.param<std::string>("tf_prefix", tf_prefix_, "");
		ROS_WARN_STREAM("EncodersController tf_prefix:" << tf_prefix_ << " ns:" << n_.getNamespace());

		std::string joint_wheel_enc;
		if (!n_.getParam("wheel_encoder_joint", joint_wheel_enc)) {
			ROS_ERROR("No joint given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_wheel_encoder_ = robot_->getJointState(/*tf_prefix_ +*/ joint_wheel_enc);

		std::string joint_steering_enc;
		if (!n_.getParam("steering_encoder_joint", joint_steering_enc)) {
			ROS_ERROR("No joint given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		joint_state_steering_encoder_ = robot_->getJointState(/*tf_prefix_ +*/ joint_steering_enc);

		if (steering_wheel_) {
			std::string joint_steering_wheel;
			if (!n_.getParam("steering_wheel_joint", joint_steering_wheel)) {
				ROS_ERROR("No joint given in namespace: '%s')", n_.getNamespace().c_str());
				return false;
			}
			joint_state_steering_wheel_ = robot_->getJointState(/*tf_prefix_ +*/ joint_steering_wheel);
		}

		return true;
	}
};
}
/*
PLUGINLIB_DECLARE_CLASS(
		lrm_gazebo,
		EncodersControllerPlugin,
		lrm_gazebo::EncodersController,
		pr2_controller_interface::Controller)
*/
PLUGINLIB_EXPORT_CLASS(
		lrm_gazebo::EncodersController,
		pr2_controller_interface::Controller
		)
