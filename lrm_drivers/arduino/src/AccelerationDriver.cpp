/*
 * AccelerationDriver.cpp
 *
 *  Created on: 19/11/2010
 *      Author: leandro
 */

#include "AccelerationDriver.h"
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include <stdio.h>

#define _DEBUG_ 0 //0 = off | 1 = on

/**
 * Constructor.
 * Create a serial communication channel and clear all variables that holds error messages and transactions responses.
 */
AccelerationDriver::AccelerationDriver() {
	serialPort = new SerialComm();
	this->lastResponseMsg[0] = '\0';
	this->erroMsg[0] = '\0';
}

/**
 * Dispose and release all resources.
 */
AccelerationDriver::~AccelerationDriver() {
	disconnect();
	delete (serialPort);
}

/**
 * Open serial port to communicate with acceleration firmware.
 * @param[in] port_name the string that describe in which the Acceleration System is attached.
 * \return -1 if fail or any other value in success case
 */
int AccelerationDriver::connect(const char *port_name) {
	if (serialPort == NULL)
		serialPort = new SerialComm();

	return serialPort->openPort(port_name);
}

/**
 * Close communication port.
 * Set velocity to zero, close the serial port communication and put the Aduino in a waiting connection state.
 */
void AccelerationDriver::disconnect() {
	if (serialPort != NULL)
		serialPort->closePort();
}

/**
 * Perform the transaction for a command (send/receive).
 * @param[in] cmd an array that describe what command is and all related data, if necessary.
 * @param[in] cmd_len command length (in bytes).
 * @param[out] resp after execution, holds the transaction response.
 * @param[in] resp_size length of response buffer.
 * @param[in] resp_len response length for this specific command.
 * \return 1 in success case or 0 if fail.
 */
int AccelerationDriver::transaction(unsigned char *cmd, int cmd_len, unsigned char resp[], int resp_size, int resp_len) {

	if (serialPort->send((void *) cmd, cmd_len) < cmd_len)
		strcpy(this->erroMsg, "Sent a truncated message");

	memset((void *) resp, 0, resp_size);

	int nbytes = 1, bytes = 0;
	while ((bytes < resp_len) && (nbytes > 0)) {
		nbytes = serialPort->receive((unsigned char *) &resp + bytes, resp_len - bytes);
		bytes += nbytes;
	}

	//Although resp can be bigger than 80 characters, lastResponseMsg won't be. So it's necessary to verify before to copy
	if (resp_size >= 80) {
		strncpy(this->lastResponseMsg, (char *)&resp, 79);
		this->lastResponseMsg[79] = '\0';
	} else
		strcpy(this->lastResponseMsg, (char *)&resp);

	return (bytes != resp_len) ? 0 : 1;
}

/**
 * Set velocity to a given value.
 * @param[in] velocity the angle value that correspond to desired velocity.
 * \return  1 if success or 0 if fail.
 */
int AccelerationDriver::setVelocity(unsigned char velocity) {
	unsigned char cmd[] = { CMD_SETVEL, velocity };
	unsigned char resp[40];

	int hadSuccess = this->transaction(cmd, 2, resp, 40, 25);

	if (_DEBUG_) printf("DEBUG: setVelocity [%s]\n", resp);

	return hadSuccess;
}

/**
 * Query what is the current velocity.
 * \return the angle value that correspond to current velocity or -1 in case of failure.
 */
int AccelerationDriver::getVelocity() {
	unsigned char cmd[] = { CMD_GETVEL };
	unsigned char resp[40];

	int hadSuccess = this->transaction(cmd, 1, resp, 40, 28);

	if (_DEBUG_) printf("DEBUG: getVelocity [%s]\n", resp);

	return (hadSuccess) ? -1 : resp[25];
}


/**
 * Send a command to Arduino reset the servo motor.
 * \return 1 in success case or 0 if fail.
 */
int AccelerationDriver::resetServo() {
	unsigned char cmd[] = { CMD_SERVORESET };
	unsigned char resp[40];

	int hadSuccess = this->transaction(cmd, 1, resp, 40, 35);

	if (_DEBUG_) printf("DEBUG: resetServo [%s]\n", resp);

	return hadSuccess;
}

/**
 * Send a command to Arduino shutdown the serial communication and rearm, putting the system in connect waiting state.
 * \return 1 in success case or 0 if fail.
 */
int AccelerationDriver::serialShutdownAndRearm() {
	unsigned char cmd[] = { CMD_SERIALRESET };
	unsigned char resp[45];

	int hadSuccess = this->transaction(cmd, 1, resp, 45, 41);

	if (_DEBUG_) printf("DEBUG: serialShutdownAndRearm [%s]\n", resp);

	return hadSuccess;
}

/**
 * Get the received message of the last command sent.
 * \return a string contains the last received message.
 */
char *AccelerationDriver::getReceivedMessage() {
	return this->lastResponseMsg;
}

/**
 * Get the error message of the last operation.
 * \return a string contains the last error message.
 */
char *AccelerationDriver::getErrorMessage() {
	return this->erroMsg;
}

/**
 * Get the instance that represents the serial communication channel
 * \return SerialComm instance ou NULL if not exist
 */
SerialComm *AccelerationDriver::getSerialComm() {
	return serialPort;
}
