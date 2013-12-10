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
 * @file imu_odometry.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 16, 2012
 *
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "lrm_odometry/OdometryConfig.h"
#include "lrm_odometry/WheelOdometry.h"
#include "lrm_odometry/WheelOdometryExtended.h"
#include "lrm_odometry/SetPose.h"

WheelOdometry* odometry;

void reconfig(lrm_odometry::OdometryConfig &config, uint32_t level)
{
	odometry->reconfigure(config, level);
}

bool poseService(lrm_odometry::SetPose::Request& req, lrm_odometry::SetPose::Response& res)
{
	odometry->setPose(req.pose.x, req.pose.y, req.pose.theta);
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "extended_odometry");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	odometry = new WheelOdometryExtended(nh);

	dynamic_reconfigure::Server<lrm_odometry::OdometryConfig> srv;
	dynamic_reconfigure::Server<lrm_odometry::OdometryConfig>::CallbackType f = boost::bind(&reconfig, _1, _2);
	srv.setCallback(f);

	//ros::ServiceServer service = nh.advertiseService(nh_priv.getNamespace() + "/set_pose", poseService);
	auto service = nh.advertiseService(nh_priv.getNamespace() + "/set_pose", poseService);

	ros::spinOnce();
	odometry->start();

	ros::spin();

	return 0;
}
