/*
 * main.cpp
 *
 *  Created on: 19/11/2010
 *      Author: leandro
 */

#include "AccelerationDriver.h"
#include <iostream>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#define DEFAULT_PORT "/dev/ttyUSB0" /**< define standard serial port for use */

/**
 * Main program.
 * This is a simple example to explain how to operate with Arduino Acceleration System
 */
int main(int argc, char **argv) {

	AccelerationDriver accel;
	int ret;

	if (argc < 2)
		ret = accel.connect(DEFAULT_PORT);
	else
		ret =accel.connect(argv[1]);

	if (ret < 0) {
		std::cout << "Problems when try to open serial port. See error messages to identify the problem nature" << std::endl;
		return -1;
	}

	accel.getSerialComm()->configure();

	accel.resetServo();
	std::cout << "So far, so good." << std::endl;

	int op, vel;

	do {
		std::cout << "Choose one of this options: " << std::endl;
		std::cout << "1: Set velocity" << std::endl;
		std::cout << "2: Get the current velocity" << std::endl;
		std::cout << "3: Reset servo" << std::endl;
		std::cout << "4: Shutdown serial and rearm" << std::endl;
		std::cout << "5: Quit" << std::endl;
		std::cout << "Option: ";
		std::cin >> op;
		std::cout << std::endl;

		switch (op) {
		case 1:
			std::cout << "Enter a int value and hit enter to set a new velocity: ";
			std::cin >> vel;
			accel.setVelocity(vel);
			//uncomment next line if you desire to see the returned bytes from serial
			//std::cout << accel.getReceivedMessage();
			break;
		case 2:
			std::cout << "Velocity: " << accel.getVelocity() << std::endl;
			//uncomment next line if you desire to see the returned bytes from serial
			//std::cout << accel.getReceivedMessage();
			break;
		case 3:
			accel.resetServo();
			//uncomment next line if you desire to see the returned bytes from serial
			//std::cout << accel.getReceivedMessage();
			break;
		case 4:
			accel.serialShutdownAndRearm();
			//uncomment next line if you desire to see the returned bytes from serial
			//std::cout << accel.getReceivedMessage();
			break;
		case 5:
			break;
		default:
			std::cout << "invalid option, try again." << std::endl;
		}
	} while (op != 5);

	accel.disconnect();
	std::cout << "bye bye :)" << std::endl;

	return 0;
}
