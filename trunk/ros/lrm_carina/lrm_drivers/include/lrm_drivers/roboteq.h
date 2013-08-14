/*
 * roboteq.h
 *
 *  Created on: 27/09/2011
 *      Author: leandro
 */

#ifndef ROBOTEQ_H_
#define ROBOTEQ_H_

#include "serial.h"
#include <iostream>
#include <string>
#include <vector>
#include <queue>


/**
 * RoboteQ is an abstract class ...
 */
class RoboteQ {

protected:
	Serial *serial_comm;			/**< An object to proceed the serial communication. */

	std::string model;				/**< Device model (AX2550, HDC2450, etc) */
	std::string errorMsg;			/**< Last error message. */
	std::string lastResponse;		/**< The module response of the last command. */

	std::vector<std::string> commandProtocol;	/**< Holds each recognized command string by control module. */
	std::vector<int> respCommandProtocol;		/**< Holds number of responses(strings) for each recognized command string by control module. */

	int useClockwiseDirection;		/**< The direction of steering control (clockwise or counterclockwise). */
	int steeringMotorIndex;			/**< Index of steering motor (where it is connected to roboteq device). */
	int brakeMotorIndex;			/**< Index of brake motor (where it is connected to roboteq device). */

	double steeringAngleRatio;     /**< motor angle/force ratio */

	pthread_t queue_thread;
	static pthread_mutex_t mutexQueueMsg;
	static std::queue<std::string> queue_messages;

	static void *incomingMessagesThread(void *arg);
	void transaction(std::string command, int howManyRepliesForThisCommand);

public:
	RoboteQ();
	virtual ~RoboteQ();

	virtual void connect(const char *port_name);
	virtual void configure() = 0;

	std::string getModel();
	std::string getErrorMsg();
	std::string getLastResponse();

	inline void setSteeringEncoderRatio(double value) {steeringAngleRatio = value;};

	void changeChannels();
	void changeSteeringDirection();

	virtual void steering(double value) { };
	virtual double getSteering() { return 0; };

	virtual void brake(signed int value) { };
	virtual long int getBrake()  { return 0; };

	virtual long int getEncoderValue( int, int = 0) { return 0; }
};

#endif /* ROBOTEQ_H_ */
