/*
 * AccelerationDriver.h
 *
 *  Created on: 19/11/2010
 *      Author: leandro
 */

#ifndef ACCELERATIONDRIVER_H_
#define ACCELERATIONDRIVER_H_

#include "SerialComm.h"

#define MAXIMUM_SKIPED_BYTES 200

/**
 * \class AccelerationDriver
 * Used to control and communicate the Arduino Acceleration System
 */
class AccelerationDriver {

private:
	SerialComm *serialPort;		/**< Serial channel with Arduino System (represent the serial port where it's attached) */
	char erroMsg[80];			/**< Holds the last error message */
	char lastResponseMsg[80];	/**< Holds the response message of the last sent command */
	int transaction(unsigned char *cmd, int cmd_len, unsigned char resp[], int resp_size, int resp_len);
public:
	/**
	 * \enum Command
	 * Define all commands supported by Arduino Acceleration System
	 */
	enum Command {
		CMD_SETVEL = 0xF5,		/**< Used to define velocity */
		CMD_GETVEL = 0xF6,		/**< Query the current velocity */
		CMD_SERVORESET = 0xF7,	/**< Reset servo position and pins */
		CMD_SERIALRESET = 0xF8	/**< Shutdown and rearms the serial Arduino port */
	};
	AccelerationDriver();
	virtual ~AccelerationDriver();
	int connect(const char *port_name);
	void disconnect();
	int setVelocity(unsigned char velocity);
	int getVelocity();
	int resetServo();
	int serialShutdownAndRearm();
	char *getErrorMessage();
	char *getReceivedMessage();
	void printSerialStatus();
	SerialComm *getSerialComm();
};


#endif /* ACCELERATIONDRIVER_H_ */
