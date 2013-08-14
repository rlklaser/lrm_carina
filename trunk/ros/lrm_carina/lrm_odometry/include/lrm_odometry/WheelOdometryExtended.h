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
 * @file WheelOdometryExtended.h
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 16, 2012
 *
 */

#ifndef WHEELODOMETRYEXTENDED_H_
#define WHEELODOMETRYEXTENDED_H_

#include "lrm_odometry/WheelOdometry.h"

class WheelOdometryExtended : public WheelOdometry
{
public:
	WheelOdometryExtended(ros::NodeHandle& n) : WheelOdometry(n) {};

private:
	virtual void calcOdometry();
};

#endif /* WHEELODOMETRYEXTENDED_H_ */
