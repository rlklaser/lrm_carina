/*
 * CANnode.h
 *
 *  Created on: Feb 8, 2013
 *      Author: mac
 */

#ifndef CANNODE_H_
#define CANNODE_H_

#include "lrm_drivers/cCAN.h"
#include <ros/ros.h>

class CAN_node {

protected:
	ros::NodeHandle nh, nh_priv;
	ros::Publisher can_publisher;
	ros::Timer canTimer;

	std::string canPath;

	cCAN* can;
	double freq;
	int baud;
	long int msg_counter;

public:
	CAN_node(ros::NodeHandle n);
	void statePublisherCallback(const ros::TimerEvent&);
	virtual ~CAN_node();
};

#endif /* CANNODE_H_ */
