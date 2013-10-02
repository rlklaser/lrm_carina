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
 * @file SuspensionController.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Dec 17, 2012
 *
 */

#include <ros/ros.h>

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/robot.h>

#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

namespace lrm_gazebo {

typedef pr2_mechanism_model::RobotState RobotState;
typedef pr2_mechanism_model::JointState JointState;
typedef std::vector<JointState*> JointStateVector;

class SuspensionController: public pr2_controller_interface::Controller {
public:

	bool init(RobotState* robot, ros::NodeHandle& n) {
		robot_ = robot;
		return loadJoints(n);
	}

	void starting() {
		ROS_INFO_STREAM("Suspension controller starting...");
	}

	void stopping() {
		ROS_INFO_STREAM("Suspension controller stopping...");
	}

	void update() {
		ros::Time t = robot_->getTime();
		joint_state_->commanded_effort_ = effort_;
	}

private:

	RobotState* robot_;
	JointState* joint_state_;
	double init_pos_;
	double effort_;

	bool loadJoints(ros::NodeHandle& n_) {

		std::string tf_prefix_;
		std::string joint;

		n_.param<std::string>("tf_prefix", tf_prefix_, "");
		ROS_INFO_STREAM("SuspensionController tf_prefix:" << tf_prefix_ << " ns:" << n_.getNamespace());

		if (!n_.getParam("joint", joint)) {
			ROS_ERROR("No joint given in namespace: '%s')", n_.getNamespace().c_str());
			return false;
		}
		ROS_INFO_STREAM("loaded suspension on joint " << joint);
		joint_state_ = robot_->getJointState(/*tf_prefix_ +*/ joint);

		n_.param("effort", effort_, 1.0);

		return true;
	}
};
}

PLUGINLIB_EXPORT_CLASS( lrm_gazebo::SuspensionController, pr2_controller_interface::Controller)
