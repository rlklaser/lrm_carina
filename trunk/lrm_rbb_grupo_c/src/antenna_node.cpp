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
 * @file antenna_node.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 29, 2012
 *
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <ns3/ptr.h>
#include <ns3/core-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/jakes-propagation-loss-model.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/buildings-propagation-loss-model.h>
#include <ns3/propagation-environment.h>
#include <ns3/yans-wifi-phy.h>

#include "lrm_rbb_grupo_c/Signal.h"

using namespace ns3;

Ptr<ConstantPositionMobilityModel> _a;
Ptr<ConstantPositionMobilityModel> _b;
Ptr<PropagationLossModel> _model;
Ptr<YansWifiPhy> _wifi_phy;

ros::Publisher _signal_pub;
std::string _ssid;
double _dBm;
double _dBi;

void callbackPose(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	_b->SetPosition(Vector(msg->x, msg->y, 0.0));
	double rssi = _model->CalcRxPower(_dBm, _b, _a);
	lrm_rbb_grupo_c::Signal sig;

	sig.header.stamp = ros::Time::now();
	sig.header.frame_id = "antenna_link";
	sig.rssi = rssi;
	sig.dBi = _dBi;
	sig.dBm = _dBm;
	sig.ssid = _ssid;

	_signal_pub.publish(sig);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "antenna_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");


    double pos_x;
    double pos_y;

    pn.param("pos_x", pos_x, 0.0);
    pn.param("pos_y", pos_y, 0.0);
    pn.param("ssid", _ssid, std::string("robombeiros"));
    pn.param("dBm", _dBm, 17.5);
    pn.param("dBi", _dBi, 0.0);

    ROS_INFO("Antenna note started");

	_a = CreateObject<ConstantPositionMobilityModel>();
	_b = CreateObject<ConstantPositionMobilityModel>();

	//ItuR1411LosPropagationLossModel
	_model = CreateObject<LogDistancePropagationLossModel>();
	_wifi_phy = CreateObject<YansWifiPhy>();

	_a->SetPosition(Vector(pos_x, pos_y, 0.0));

	_signal_pub = n.advertise<lrm_rbb_grupo_c::Signal>(n.getNamespace() + "/signal", 2);
	ros::Subscriber pose_sub = n.subscribe<geometry_msgs::Pose2D>(n.getNamespace() + "/ground_truth", 2, &callbackPose);

	ros::spin();
}
