/*
 * vcmdas1_throttle.h
 *
 *  Created on: Jan 27, 2013
 *      Author: Diego Gomes
 */

#ifndef ARDUINO_THROTTLE_H
#define ARDUINO_THROTTLE_H


#include "lrm_drivers/serial.h"
#include "lrm_drivers/throttle.h"
#include <pthread.h>
#include <queue>
#include <string>
#include <cstdlib>
#include <cstdio>
#include <cassert>
#include <iostream>


#define ARDUINO_DEFAULT_CHAR_DEVICE "/dev/ttyUSB0"
#define ARDUINO_DEFAULT_BAUD 9600
#define SIZE_BUFF 256

#define CMD_SETVEL      0xF5
#define CMD_STATUS      0xF6
#define CMD_SERVORESET  0xF7
#define CMD_RESET       0xF8
#define CMD_PING        0xF9
#define CMD_PONG        0xFA

class ArduinoThrottle : public Throttle {
private:

protected:
	int arduinoBaud; 			/**< Serial communication baudrate */
	std::string arduinoPath;	/**< Acceleration device path */
	Serial *arduino; 			/**< Serial port communication to Arduino device */

	pthread_t queue_thread;

	static void *incomingMessagesThread(void *arg) ;


public:
	ArduinoThrottle();
	ArduinoThrottle( const int& min, const int&  max, const std::string& arduinoPath, const int& arduinoBaud);
	virtual ~ArduinoThrottle();
	void setAccel (const double& accel);
	const double& getAccel()const;



};




#endif /** ARDUINO_THROTTLE_H**/
