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
 * @file carina_controller.h
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 25, 2012
 *
 */

#ifndef CARINA_CONTROLLER_H_
#define CARINA_CONTROLLER_H_

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>

namespace carina_controller
{

/**
 * Gazebo controller class that emulates the robot controllability:
 * 	- encoders
 * 	- actuators
 *
 * Works like a simulated driver of the real robot
 */

class CarinaControllerClass: public pr2_controller_interface::Controller
{

private:
	pr2_mechanism_model::JointState* joint_state_flw_dir_; //front left wheel direction
	pr2_mechanism_model::JointState* joint_state_frw_dir_; //front right wheel direction
	pr2_mechanism_model::JointState* joint_state_flw_; //front left wheel
	pr2_mechanism_model::JointState* joint_state_frw_; //front right wheel
	pr2_mechanism_model::JointState* joint_state_blw_; //rear left wheel
	pr2_mechanism_model::JointState* joint_state_brw_; //rear right wheel
	pr2_mechanism_model::JointState* joint_state_stw_; //steering wheel

protected:

public:
	CarinaControllerClass();
	virtual ~CarinaControllerClass();

}

#endif /* CARINA_CONTROLLER_H_ */
