/*
 * roboteq.cpp
 *
 *  Created on: 27/09/2011
 *      Author: leandro
 */

#include "lrm_drivers/roboteq.h"
#include <cstring>
#include <ros/ros.h>

#define DEBUG false				/**< Use true or false to activate the debug mode */
#define _DEBUG_MSG_ if (DEBUG)

#define SIZE_BUFF 256

pthread_mutex_t RoboteQ::mutexQueueMsg;
std::queue<std::string> RoboteQ::queue_messages;

/**
 * This method operates like a receiver listener, parsing each transmitted message and storing into
 * a data structure.
 * @param arg NULL, for this propose.
 * @return NULL, for this propose.
 */
void *RoboteQ::incomingMessagesThread(void *arg) {
	RoboteQ *instance = (RoboteQ*) arg;
	char buff[SIZE_BUFF];
	int acc, nBytesRead;

	while (true) {
		acc = 0;
		memset( (void*)buff, 0, SIZE_BUFF );
		do {
			nBytesRead = instance->serial_comm->receive((void *)(buff+acc), SIZE_BUFF);
			acc += nBytesRead;
		} while ( buff[acc-1] != 0x0a );
		
		char *pstr, *str = buff;
		pstr = strtok_r(str, "\n", &str);

		while( pstr != 0 ) {
			pthread_mutex_lock( &mutexQueueMsg );
			queue_messages.push( std::string(pstr) );
			pthread_mutex_unlock( &mutexQueueMsg );
			pstr = strtok_r(str, "\n\0", &str);
		}
	}

	return NULL;
}

/**
 * Default constructor.
 * Initiate the mainly properties and create a serial object for communication.
 */
RoboteQ::RoboteQ() {
	steeringMotorIndex    = 1;
	brakeMotorIndex       = 0;
	useClockwiseDirection = 0;

	model = std::string("none");
	errorMsg = std::string("no error");

	serial_comm = new Serial();
}

/**
 * Default destructor.
 * Release all resources related to serial object.
 */
RoboteQ::~RoboteQ() {
	delete serial_comm;
}

/**
 * This method establish a serial communication channel with a device attached to port_name.
 * @param[in] port_name Where the device is attached (e.g. /dev/ttyUSB0).
 */
void RoboteQ::connect(const char *port_name) {
	serial_comm->openPort(port_name);
	this->configure();

	//mutexQueueMsg = PTHREAD_MUTEX_INITIALIZER;
	pthread_create(&queue_thread, NULL, &incomingMessagesThread, this);
}

/**
 * Query what is the specific model of device.
 * @return device model (e.g. AX2550)
 */
std::string RoboteQ::getModel() {
	return model;
}

/**
 * Get the last response from controller module.
 * @return String returned by controller.
 */
std::string RoboteQ::getLastResponse() {
	return this->lastResponse;
}

/**
 * Holds any error message that was occurred in some previous transaction.
 * @return The error message.
 */
std::string RoboteQ::getErrorMsg() {
	return this->errorMsg;
}

/**
 * Perform a send and receive transaction with controller module, giving a command
 */
void RoboteQ::transaction(std::string command, int howManyRepliesForThisCommand) {

	char cmd[64];
	int cmd_len = command.length();
	
	strncpy( cmd, command.c_str(), cmd_len );
	cmd[cmd_len] = 0x0D;
	cmd_len++;

	int nBytesSent = serial_comm->send((void *) cmd, cmd_len);
	if (nBytesSent != cmd_len) {
		this->errorMsg.assign("The sent message was truncated.");
	}

	lastResponse.clear();
	int resp_count = 0;
	while (resp_count < howManyRepliesForThisCommand) {
		pthread_mutex_lock( &mutexQueueMsg );
		if (!queue_messages.empty()) {
			lastResponse.append( queue_messages.front() );
			queue_messages.pop();
			resp_count++;
			lastResponse.append( "\n" );
		}
		pthread_mutex_unlock( &mutexQueueMsg );
	}

}

/**
 * Change logically the channels of two motor at controller.
 * Allow a logic interchange between the two motor control channels.
 */
void RoboteQ::changeChannels() {
	brakeMotorIndex = steeringMotorIndex;
	steeringMotorIndex = ( steeringMotorIndex + 1 ) % 2;
}

/**
 * Sets if the controller will operate a clockwise or counterclockwise mode.
 */
void RoboteQ::changeSteeringDirection() {
	this->useClockwiseDirection = (this->useClockwiseDirection) ? 0 : 1;
}
