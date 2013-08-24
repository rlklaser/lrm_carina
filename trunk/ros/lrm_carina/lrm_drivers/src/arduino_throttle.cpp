/*
 * vcmdas1_throttle.h
 *
 *  Created on: Jan 27, 2013
 *      Author: Diego Gomes
 */

#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include "lrm_drivers/serial.h"
#include <ros/ros.h>
#include "lrm_drivers/arduino_throttle.h"

ArduinoThrottle::ArduinoThrottle() :
		Throttle() {

	this->arduinoPath = ARDUINO_DEFAULT_CHAR_DEVICE;
	this->arduinoBaud = ARDUINO_DEFAULT_BAUD;
	this->arduino = new Serial();
	this->arduino->openPort(this->arduinoPath.c_str());
	this->arduino->configure(this->arduinoBaud, 8, false, 0, 0, 1, 1, 0);
	pthread_create(&queue_thread, NULL, &incomingMessagesThread, this);
	setAccel(0);
}

ArduinoThrottle::ArduinoThrottle(const int& min, const int& max, const std::string& arduinoPath, const int& arduinoBaud) :
		Throttle(min, max) {

	this->arduinoBaud = arduinoBaud;
	this->arduinoPath = arduinoPath;
	this->arduino = new Serial();
	this->arduino->openPort(this->arduinoPath.c_str());
	this->arduino->configure(this->arduinoBaud, 8, false, 0, 0, 1, 1, 0);
	pthread_create(&queue_thread, NULL, &incomingMessagesThread, this);
	setAccel(0);
}

ArduinoThrottle::~ArduinoThrottle() {
	setAccel(0);
	ROS_INFO_STREAM("arduino clean exit");
	delete this->arduino;
}

void ArduinoThrottle::setAccel(const double& accel) {
	if (!this->arduino->isConnected()) {
		throw std::exception();
		return;
	}
	if (this->currentAccel != accel) { //only send to hw if changed
		this->currentAccel = accel;
		char value = (char) (this->currentAccel * (this->max_value - this->min_value) / 100);
		char msg[2] = { CMD_SETVEL, value };
		this->arduino->send((void *) msg, 2);
	}
}

const double& ArduinoThrottle::getAccel() const {
	return this->currentAccel;
}

void *ArduinoThrottle::incomingMessagesThread(void *arg) {
	ArduinoThrottle *instance = (ArduinoThrottle *) arg;
	char buff[SIZE_BUFF];
	int acc, nBytesRead;

	while (true) {
		acc = 0;
		memset((void*) buff, 0, SIZE_BUFF);
		do {
			nBytesRead = instance->arduino->receive((void *) (buff + acc), SIZE_BUFF);
			acc += nBytesRead;
		} while (buff[acc - 1] != 0x0a);

	}

	return NULL;
}
