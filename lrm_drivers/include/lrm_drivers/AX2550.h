/*
 * AX2550.h
 *
 *  Created on: 27/09/2011
 *      Author: leandro
 */

#ifndef AX2550_H_
#define AX2550_H_

#include "roboteq.h"

#define ENCODER_ABS 0
#define ENCODER_REL 1

class AX2550: public RoboteQ {

protected:

public:
	enum Commands {				/**< Represents the device protocol */
		RESET_DEVICE,
		GET_FIRST_ENCODER_ABS,
		GET_SECOND_ENCODER_ABS,
		GET_FIRST_ENCODER_REL,
		GET_SECOND_ENCODER_REL,
		SET_MOTOR_CHANNEL_1_FWD,
		SET_MOTOR_CHANNEL_1_REV,
		SET_MOTOR_CHANNEL_2_FWD,
		SET_MOTOR_CHANNEL_2_REV,
		RESET_FIRST_ENCODER,
		RESET_SECOND_ENCODER,
		RESET_BOTH_ENCODER,
		READ_BATTERY_AMPS,
		READ_POWER_LEVEL,
		READ_ANALOG_INPUTS_1_AND_2,
		READ_ANALOG_INPUTS_3_AND_4
	};

	

	AX2550();
	virtual ~AX2550();

	void configure();

	std::string composeCmdWithHex( std::string cmd, int value );
	void execute(Commands cmd, int value);

	void steering(double value);
	double getSteering();
	void brake(signed int value);
	long int getBrake();

	long int getEncoderValue( int, int );
};

#endif /* AX2550_H_ */
