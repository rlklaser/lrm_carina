/*
 * CANnode.cpp
 *
 *  Created on: Feb 8, 2013
 *      Author: mac
 */

#include "lrm_drivers/can_node.h"
#include "lrm_drivers/cCAN.h"
#include "lrm_msgs/VehicleState.h"
#include <ros/ros.h>

CAN_node::CAN_node(ros::NodeHandle n):
	nh(n), nh_priv("~") {

	nh_priv.param("port", this->canPath, std::string("/dev/ttyUSB0") );
	nh_priv.param<double>("which_frequency", this->freq, 5.0);
	nh_priv.param("baud", this->baud, B921600);

	can =  new cCAN(canPath.c_str(), baud);
	this->msg_counter = 0;

	can_publisher = nh.advertise<lrm_msgs::VehicleState>(std::string("vehicle_state"), 10, false);
	canTimer = n.createTimer(ros::Duration(1.0/freq), &CAN_node::statePublisherCallback, this);
}

void CAN_node::statePublisherCallback(const ros::TimerEvent&) {
	lrm_msgs::VehicleState status;

	status.header.frame_id = "/vehicle_state";
	status.header.seq = msg_counter++;
	status.header.stamp = ros::Time::now();
	status.velocity = can->getVelocity();

	can_publisher.publish(status);
}

CAN_node::~CAN_node() {
	delete can;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "CAN");
	ros::NodeHandle n;

	CAN_node can_node(n);

	ros::spin();
}
