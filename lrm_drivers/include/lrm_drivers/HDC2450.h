/*
 * HDC2450.h
 *
 *  Created on: 05/10/2011
 *      Author: leandro
 */

#ifndef HDC2450_H_
#define HDC2450_H_

#include "roboteq.h"

class HDC2450: public RoboteQ {

protected:
	std::string composeCmd( std::string cmd, int value );
	std::string composeCmd( std::string cmd, int motorindex, int value );

public:
	enum Commands {
		RESET_DEVICE,
		SET_ACCELERATION,
		SET_DECELERATION,
		SET_ALL_DIGITAL_OUT_BITS,
		RESET_INDIVIDUAL_OUT_BITS,
		SET_INDIVIDUAL_OUT_BITS,
		EMERGENCY_SHUTDOWN,
		SET_MOTOR_COMMAND,
		LOAD_HOME_COUNTER,
		RELEASE_SHUTDOWN,
		SET_CMD_FOR_1_OR_2_CHANNELS,
		SET_POSITION,
		SET_VELOCITY,
		SET_BRUSHLESS_COUNTER,
		SET_ENCODER_COUNTER,
		GET_ENCODER_COUNTER
	};
	HDC2450();
	virtual ~HDC2450();

	void configure();

	void execute(Commands cmd, int firstParam = -1, int secondParam = -1);

	void steering(double value);
	double getSteering();

	void brake(signed int value);
	long int getBrake();
	long int getEncoderValue(int index, int mode);
};

#endif /* HDC2450_H_ */
