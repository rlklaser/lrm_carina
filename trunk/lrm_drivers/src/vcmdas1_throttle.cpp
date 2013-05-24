/*
 * vcmdas1_throttle.cpp
 *
 *  Created on: Jan 28, 2013
 *      Author: Diego Gomes
 */

#include "lrm_drivers/vcmdas1_throttle.h"
#include "lrm_drivers/vcmdas1_driver.h"
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <ros/ros.h>

VcmDas1Throttle::VcmDas1Throttle() :
		Throttle() {
	this->card = new Vcmdas1(std::string("/dev/vcmdas1"), 0X300, 10, 5, 5);
	this->channel = 1;
	this->setAccel(0.0);
}

VcmDas1Throttle::VcmDas1Throttle(const int& max,
		const std::string& charDevicePath, const unsigned int& base_addr,
		const unsigned int& ch) :
		Throttle(0, max) {
	this->card = new Vcmdas1(charDevicePath, 0X300, 10, 5, 5);
	this->channel = ch;
	this->setAccel(0.0);
}

VcmDas1Throttle::~VcmDas1Throttle() {
	delete card;
}

void VcmDas1Throttle::setAccel(const double& accel) {
	unsigned int reg16;
	//ROS_INFO("VcmDas1Throttle::setAccel   set to %d max=%d", accel,max_value);
	this->currentAccel = accel;

	if (this->currentAccel > this->max_value) {
		reg16 = MIN_VALUE_DAC;
		card->analogOut(channel,reg16);
		throw std::bad_alloc();
		return;
	}

	reg16 = MIN_VALUE_DAC
			+ (((double) (MAX_VALUE_DAC - MIN_VALUE_DAC) / 100)
					* this->currentAccel);
	card->analogOut(channel,reg16);
}

const double& VcmDas1Throttle::getAccel() const {
	return this->currentAccel;
}

