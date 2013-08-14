/*
 * HDC2450.cpp
 *
 *  Created on: 05/10/2011
 *      Author: leandro
 */

#include "lrm_drivers/HDC2450.h"
#include <vector>
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

HDC2450::HDC2450() {

	model = std::string("HDC2450");

	commandProtocol.push_back( std::string("%RESET 321654987") );//RESET_DEVICE
	commandProtocol.push_back( std::string("!AC nn mm") );	//SET_ACCELERATION
	commandProtocol.push_back( std::string("!DC nn mm") );	//SET_DECELERATION
	commandProtocol.push_back( std::string("!DS nn") );		//SET_ALL_DIGITAL_OUT_BITS
	commandProtocol.push_back( std::string("!D0 nn") );		//RESET_INDIVIDUAL_OUT_BITS
	commandProtocol.push_back( std::string("!D0 nn") );		//SET_INDIVIDUAL_OUT_BITS
	commandProtocol.push_back( std::string("!EX") );		//EMERGENCY_SHUTDOWN
	commandProtocol.push_back( std::string("!G nn mm") );	//SET_MOTOR_COMMAND
	commandProtocol.push_back( std::string("!H nn") );		//LOAD_HOME_COUNTER
	commandProtocol.push_back( std::string("!MG") );		//RELEASE_SHUTDOWN
	commandProtocol.push_back( std::string("!M nn mm") );	//SET_CMD_FOR_1_OR_2_CHANNELS
	commandProtocol.push_back( std::string("!P nn mm") );	//SET_POSITION
	commandProtocol.push_back( std::string("!S nn mm") );	//SET_VELOCITY
	commandProtocol.push_back( std::string("!CB nn") );		//SET_BRUSHLESS_COUNTER
	commandProtocol.push_back( std::string("!C nn mm") );	//SET_ENCODER_COUNTER
	commandProtocol.push_back( std::string("?C nn") );		//GET_ENCODER_COUNTER

	respCommandProtocol.push_back( 3 );	//RESET_DEVICE
	respCommandProtocol.push_back( 2 );	//SET_ACCELERATION
	respCommandProtocol.push_back( 2 );	//SET_DECELERATION
	respCommandProtocol.push_back( 2 );	//SET_ALL_DIGITAL_OUT_BITS
	respCommandProtocol.push_back( 2 );	//RESET_INDIVIDUAL_OUT_BITS
	respCommandProtocol.push_back( 2 );	//SET_INDIVIDUAL_OUT_BITS
	respCommandProtocol.push_back( 1 );	//EMERGENCY_SHUTDOWN
	respCommandProtocol.push_back( 2 );	//SET_MOTOR_COMMAND
	respCommandProtocol.push_back( 2 );	//LOAD_HOME_COUNTER
	respCommandProtocol.push_back( 1 );	//RELEASE_SHUTDOWN
	respCommandProtocol.push_back( 2 );	//SET_CMD_FOR_1_OR_2_CHANNELS
	respCommandProtocol.push_back( 2 );	//SET_POSITION
	respCommandProtocol.push_back( 2 );	//SET_VELOCITY
	respCommandProtocol.push_back( 2 );	//SET_BRUSHLESS_COUNTER
	respCommandProtocol.push_back( 2 );	//SET_ENCODER_COUNTER
	respCommandProtocol.push_back( 2 );	//GET_ENCODER_COUNTER
}

HDC2450::~HDC2450() {
}

//@pure virtual method implementation
void HDC2450::configure() {
	if (serial_comm->isConnected())
		serial_comm->configure(115200, 8, false, 0, 1, 1, 0, 5);
	else {
		errorMsg.assign("Impossible to configure RoboteQ.");
		ROS_ERROR("%s", errorMsg.c_str());
	}
}

//@override
void HDC2450::execute(Commands cmd, int firstParam, int secondParam) {
	switch (cmd) {
		case RESET_DEVICE:
			transaction( commandProtocol[RESET_DEVICE], respCommandProtocol[RESET_DEVICE]);
			break;
		case SET_ACCELERATION:
		case SET_DECELERATION:
		case SET_ALL_DIGITAL_OUT_BITS:
		case RESET_INDIVIDUAL_OUT_BITS:
		case SET_INDIVIDUAL_OUT_BITS:
		case EMERGENCY_SHUTDOWN:
			break;
		case SET_MOTOR_COMMAND:
			transaction( composeCmd(commandProtocol[SET_MOTOR_COMMAND], firstParam, secondParam), respCommandProtocol[SET_MOTOR_COMMAND]);
			break;
		case LOAD_HOME_COUNTER:
		case RELEASE_SHUTDOWN:
		case SET_CMD_FOR_1_OR_2_CHANNELS:
		case SET_POSITION:
			transaction( composeCmd(commandProtocol[SET_POSITION], firstParam, secondParam), respCommandProtocol[SET_POSITION]);
			break;
		case SET_VELOCITY:
		case SET_BRUSHLESS_COUNTER:
		case SET_ENCODER_COUNTER:
			break;
		case GET_ENCODER_COUNTER:
			transaction( composeCmd(commandProtocol[GET_ENCODER_COUNTER], firstParam), respCommandProtocol[GET_ENCODER_COUNTER]);
			break;
		default:
			this->errorMsg.assign("Command not defined.");
			ROS_ERROR("%s", errorMsg.c_str());
			break;
	}
}

/**
 * This method compose into a unique command string the correspondent roboteq code and his value
 * @param cmd one of RoboteQ commands.
 * @param value an integer value to be combined with command cmd
 * @return the full command to be used by RoboteQ.
 */
std::string HDC2450::composeCmd( std::string cmd, int value ) {
	std::string composedCmd;
	std::size_t pos;
	char str[20];

	composedCmd.assign( cmd );

	sprintf(str, "%d", value);
	pos = composedCmd.find("nn");
	composedCmd.replace(pos, 2, str);

	return composedCmd;
}


/**
 * This method compose into a unique command string the correspondent RoboteQ's command code and his values
 * @param cmd one of RoboteQ commands.
 * @param motorindex the integer that represents one of motors channel.
 * @param value an integer value to be combined with command cmd
 * @return the full command to be used by RoboteQ.
 */
std::string HDC2450::composeCmd( std::string cmd, int motorindex, int value ) {
	std::string composedCmd;
	std::size_t pos;
	char str[20];

	composedCmd.assign( cmd );

	sprintf(str, "%d", motorindex);
	pos = composedCmd.find("nn");
	composedCmd.replace(pos, 2, str);

	sprintf(str, "%d", value);
	pos = composedCmd.find("mm");
	composedCmd.replace(pos, 2, str);

	return composedCmd;
}

//@override
void HDC2450::steering(double angle) {
	//              (max left)  (max right)
	//Encoder range: -27500  <-->  27500
	//int value = (int) angle * -700;
	//execute(SET_POSITION, 1, value);

	int value = (int) angle * -33.3;
	execute(SET_MOTOR_COMMAND, 1, value);
}

//@override
double HDC2450::getSteering() {
	ROS_ERROR("HDC2450::getSteering() - NOT IMPLEMENTED!!!!!");
	return 0;
}

//@override
void HDC2450::brake(signed int value) {
	execute(SET_MOTOR_COMMAND, 2, value*10);
}

//@override
long int HDC2450::getBrake() {
	ROS_ERROR("HDC2450::getBrakeEncoderValue() - NOT IMPLEMENTED!!!!!");
	return 0;
}

//@override
long int HDC2450::getEncoderValue(int index, int mode = NULL) {
	execute(GET_ENCODER_COUNTER, index, NULL);

	char *ptr;
	ptr = strtok((char *) lastResponse.c_str(), "=");
	ptr = strtok(NULL, "=");

	return atol(ptr);
}
