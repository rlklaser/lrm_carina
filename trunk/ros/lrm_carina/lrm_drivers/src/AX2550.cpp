/*
 * AX2550.cpp
 *
 *  Created on: 27/09/2011
 *      Author: leandro
 */

#include "lrm_drivers/AX2550.h"
#include <vector>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <ros/ros.h>

/**
 * Constructor.
 * Create an specific instance of RoboteQ (model AX 2550)
 */
AX2550::AX2550() : RoboteQ() {

	model = std::string("AX2550");

	commandProtocol.push_back( std::string("%rrrrrr") );	// RESET_DEVICE
	commandProtocol.push_back( std::string("?Q0") );	// GET_FIRST_ENCODER_ABS
	commandProtocol.push_back( std::string("?Q1") );	// GET_SECOND_ENCODER_ABS
	commandProtocol.push_back( std::string("?Q4") );	// GET_FIRST_ENCODER_REL
	commandProtocol.push_back( std::string("?Q5") );	// GET_SECOND_ENCODER_REL
	commandProtocol.push_back( std::string("!Ann") );	// SET_MOTOR_CHANNEL_1_FWD
	commandProtocol.push_back( std::string("!ann") );	// SET_MOTOR_CHANNEL_1_REV
	commandProtocol.push_back( std::string("!Bnn") );	// SET_MOTOR_CHANNEL_2_FWD
	commandProtocol.push_back( std::string("!bnn") );	// SET_MOTOR_CHANNEL_2_REV
	commandProtocol.push_back( std::string("!Q0") );	// RESET_FIRST_ENCODER
	commandProtocol.push_back( std::string("!Q1") );	// RESET_SECOND_ENCODER
	commandProtocol.push_back( std::string("!Q2") );	// RESET_BOTH_ENCODER
	commandProtocol.push_back( std::string("?A") );		// READ_BATTERY_AMPS
	commandProtocol.push_back( std::string("?V") );		// READ_POWER_LEVEL
	commandProtocol.push_back( std::string("?P") );		// READ_ANALOG_INPUTS_1_AND_2
	commandProtocol.push_back( std::string("?R") );		// READ_ANALOG_INPUTS_3_AND_4

	respCommandProtocol.push_back( 6 );	// RESET_DEVICE
	respCommandProtocol.push_back( 2 );	// GET_FIRST_ENCODER_ABS
	respCommandProtocol.push_back( 2 );	// GET_SECOND_ENCODER_ABS
	respCommandProtocol.push_back( 2 );	// GET_FIRST_ENCODER_REL
	respCommandProtocol.push_back( 2 );	// GET_SECOND_ENCODER_REL
	respCommandProtocol.push_back( 2 );	// SET_MOTOR_CHANNEL_1_FWD
	respCommandProtocol.push_back( 2 );	// SET_MOTOR_CHANNEL_1_REV
	respCommandProtocol.push_back( 2 );	// SET_MOTOR_CHANNEL_2_FWD
	respCommandProtocol.push_back( 2 );	// SET_MOTOR_CHANNEL_2_REV
	respCommandProtocol.push_back( 2 );	// RESET_FIRST_ENCODER
	respCommandProtocol.push_back( 2 );	// RESET_SECOND_ENCODER
	respCommandProtocol.push_back( 2 );	// RESET_BOTH_ENCODER
	respCommandProtocol.push_back( 3 );	// READ_BATTERY_AMPS
	respCommandProtocol.push_back( 3 );	// READ_POWER_LEVEL
	respCommandProtocol.push_back( 3 );	// READ_ANALOG_INPUTS_1_AND_2
	respCommandProtocol.push_back( 3 );	// READ_ANALOG_INPUTS_3_AND_4

}

AX2550::~AX2550() {
}

//@pure virtual method implementation
void AX2550::configure() {
	if ( serial_comm->isConnected() )
		serial_comm->configure(9600,7,false,2,1,1,1,0);
	else {
		errorMsg.assign("Impossible to configure RoboteQ.");
		ROS_ERROR("%s", errorMsg.c_str());
	}
}

std::string AX2550::composeCmdWithHex( std::string cmd, int value ) {
	std::string composedCmd;
	char valueInHex[3];
	std::size_t pos;

	//ROS_ASSERT( (value >= 0) && (value <= 127) );
	if(!((value >= 0) && (value <= 127))) {
		ROS_ERROR_STREAM("roboteq: value out of range " << value);
		if(value<0) value = 0;
		if(value>127) value = 127;
	}

	composedCmd.assign( cmd );
	pos = composedCmd.find("nn");
	sprintf(valueInHex, "%02x", value);
	composedCmd.replace(pos, 2, valueInHex);
	
	return composedCmd;
}

//@override
void AX2550::execute(Commands cmd, int value) {
	switch (cmd) {
		case RESET_DEVICE:
			transaction(commandProtocol[RESET_DEVICE], respCommandProtocol[RESET_DEVICE]);
			break;
		case GET_FIRST_ENCODER_ABS:
			transaction(commandProtocol[GET_FIRST_ENCODER_ABS], respCommandProtocol[GET_FIRST_ENCODER_ABS]);
			break;
		case GET_SECOND_ENCODER_ABS:
			transaction(commandProtocol[GET_SECOND_ENCODER_ABS], respCommandProtocol[GET_SECOND_ENCODER_ABS]);
			break;
		case GET_FIRST_ENCODER_REL:
			transaction(commandProtocol[GET_FIRST_ENCODER_REL], respCommandProtocol[GET_FIRST_ENCODER_REL]);
			break;
		case GET_SECOND_ENCODER_REL:
			transaction(commandProtocol[GET_SECOND_ENCODER_REL], respCommandProtocol[GET_SECOND_ENCODER_REL]);
			break;
		case SET_MOTOR_CHANNEL_1_FWD:
			transaction( composeCmdWithHex( commandProtocol[SET_MOTOR_CHANNEL_1_FWD], value), 
					respCommandProtocol[SET_MOTOR_CHANNEL_1_FWD]);
			break;
		case SET_MOTOR_CHANNEL_1_REV:
			transaction( composeCmdWithHex( commandProtocol[SET_MOTOR_CHANNEL_1_REV], value),
					respCommandProtocol[SET_MOTOR_CHANNEL_1_REV]);
			break;
		case SET_MOTOR_CHANNEL_2_FWD:
			transaction( composeCmdWithHex( commandProtocol[SET_MOTOR_CHANNEL_2_FWD], value ),
					respCommandProtocol[SET_MOTOR_CHANNEL_2_FWD]);
			break;
		case SET_MOTOR_CHANNEL_2_REV:
			transaction( composeCmdWithHex( commandProtocol[SET_MOTOR_CHANNEL_2_REV], value ),
					respCommandProtocol[SET_MOTOR_CHANNEL_2_REV]);
			break;
		case RESET_FIRST_ENCODER:
			transaction(commandProtocol[RESET_FIRST_ENCODER], respCommandProtocol[RESET_FIRST_ENCODER]);
			break;
		case RESET_SECOND_ENCODER:
			transaction(commandProtocol[RESET_SECOND_ENCODER], respCommandProtocol[RESET_SECOND_ENCODER]);
			break;
		case RESET_BOTH_ENCODER:
			transaction(commandProtocol[RESET_BOTH_ENCODER], respCommandProtocol[RESET_BOTH_ENCODER]);
			break;
		case READ_BATTERY_AMPS:
			transaction(commandProtocol[READ_BATTERY_AMPS], respCommandProtocol[READ_BATTERY_AMPS]);
			break;
		case READ_POWER_LEVEL:
			transaction(commandProtocol[READ_POWER_LEVEL], respCommandProtocol[READ_POWER_LEVEL]);
			break;
		case READ_ANALOG_INPUTS_1_AND_2:
			transaction(commandProtocol[READ_ANALOG_INPUTS_1_AND_2], respCommandProtocol[READ_ANALOG_INPUTS_1_AND_2]);
			break;
		case READ_ANALOG_INPUTS_3_AND_4:
			transaction(commandProtocol[READ_ANALOG_INPUTS_3_AND_4], respCommandProtocol[READ_ANALOG_INPUTS_3_AND_4]);
			break;
		default:
			this->errorMsg.assign("Command not defined.");
			break;
	}
}

//@override
void AX2550::steering(double angle) {
	int value = (int) angle * steeringAngleRatio; //4.1

	if (steeringMotorIndex == 0) {
		if (useClockwiseDirection)				
			this->execute((value > 0) ? SET_MOTOR_CHANNEL_1_FWD : SET_MOTOR_CHANNEL_1_REV, abs(value));
		else
			this->execute((value > 0) ? SET_MOTOR_CHANNEL_1_REV : SET_MOTOR_CHANNEL_1_FWD, abs(value));
	}
	else {
		if (useClockwiseDirection)
			this->execute((value > 0) ? SET_MOTOR_CHANNEL_2_FWD : SET_MOTOR_CHANNEL_2_REV, abs(value));
		else
			this->execute((value > 0) ? SET_MOTOR_CHANNEL_2_REV : SET_MOTOR_CHANNEL_2_FWD, abs(value));
	}
}

//@override
double AX2550::getSteering() {
	ROS_DEBUG("AX2550::getSteering() - NOT IMPLEMENTED!!!!");
	//This method needs to call getEncoderValue() to compute steering angle from encoder value
	return 0;
}

//@override
long int AX2550::getEncoderValue( int index, int mode ) {
	Commands cmd;

	//this->execute((index == 0) ? GET_FIRST_ENCODER_ABS : GET_SECOND_ENCODER_ABS, 0);
	if(index == 0) {
		if(mode == ENCODER_ABS)
			cmd = GET_FIRST_ENCODER_ABS;
		else
			cmd = GET_FIRST_ENCODER_REL;
	}
	else {
		if(mode == ENCODER_ABS)
			cmd = GET_SECOND_ENCODER_ABS;
		else
			cmd = GET_SECOND_ENCODER_REL;
	}
	this->execute(cmd, 0);

	int pos = lastResponse.find("\n");
	std::string resp = lastResponse.substr( pos+1, lastResponse.length()-pos-2 );

	std::string aux;	
	aux.assign( (resp[0] <= '7') ? "00000000" : "7FFFFFFF" );
	int tam = resp.length();
	for (int i = 7; i > 7 - tam; i--)
		aux.at(i) = resp.at(tam - 1 + i - 7);

	int encoderValue = strtol(aux.c_str(), NULL, 16);
	if (!(resp.at(0) <= '7'))
		encoderValue -= 0x7FFFFFFF;

	return (index == 1 && useClockwiseDirection == 0)?-encoderValue:encoderValue;
}

//@override
void AX2550::brake(signed int value) {
	if (brakeMotorIndex == 0)
		this->execute((value > 0) ? SET_MOTOR_CHANNEL_1_FWD : SET_MOTOR_CHANNEL_1_REV, value);
	else
		this->execute((value > 0) ? SET_MOTOR_CHANNEL_2_FWD : SET_MOTOR_CHANNEL_2_REV, value);
}

//@override
long int AX2550::getBrake() {
	std::string aux;

	this->execute((brakeMotorIndex == 0) ? GET_FIRST_ENCODER_ABS : GET_SECOND_ENCODER_ABS, 0);

	aux.assign( (lastResponse.at(0) <= '7') ? "00000000" : "7FFFFFFF" );
	int tam = lastResponse.length();
	for (int i = 7; i > 7 - tam; i--)
		aux.at(i) = lastResponse.at(tam - 1 + i - 7);

	int encoderValue = strtol(aux.c_str(), NULL, 16);
	if (!(lastResponse.at(0) <= '7'))
		encoderValue -= 0x7FFFFFFF;

	return encoderValue;
}
