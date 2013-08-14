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
 * @file WheelOdometrySimple.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 16, 2012
 *
 */

#include "lrm_odometry/WheelOdometrySimple.h"

void WheelOdometrySimple::calcOdometry()
{
	// Compute odometry according to the kinematic model of a car-like robot.
	odo.dist_f = odo.encWheelValue * odo.p.const_odom;
	odo.phi = odo.encSteerValue * odo.p.const_steer;

	if(odo.phi!=0) {
		odo.dist_r = odo.dist_f / sqrt(1+tan(odo.phi)*tan(odo.phi));
	}
	else {
		odo.dist_r = odo.dist_f;
	}

	//middle axel distance
	odo.dist = odo.dist_r - odo.p.robot_width * odo.delta_theta / 2;

	if(odo.p.use_imu) {
		odo.delta_theta = odo.yaw - odo.last_orientation;
	} else {
		if(odo.phi!=0) {
			odo.delta_theta = odo.dist * tan(odo.phi) / odo.p.robot_length;
		}
		else {
			odo.delta_theta = 0;
		}
	}

	//just for printing
	odo.phi_l = odo.phi;
	odo.phi_r = odo.phi;
	odo.dist_f_l = odo.dist_f;
	odo.dist_f_r = odo.dist_f;
	odo.dist_r_l = odo.dist_r;
	odo.dist_r_r = odo.dist_r;
}
