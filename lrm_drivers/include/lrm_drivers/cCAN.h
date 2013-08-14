/*
 * CAN.h
 *
 *  Created on: Feb 8, 2013
 *      Author: mac
 */

#ifndef CAN_H_
#define CAN_H_

#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <time.h>

#define CAN_DEFAULT_CHAR_DEVICE "/dev/ttyUSB0"
#define CAN_DEFAULT_BAUD B921600
#define CAN_VELOCITY_AJUST 0.125
#define SIZE_BUFF 256
#define CAN_UNTIL '\n'

#define ID_VELOCIDADE 0x5A0

class cCAN {
private:
	int temp;
protected:
	int fd;
	int serialport_read(char* buf, char until);
	double calcVelocity(char* buf);
	double velocity;
	pthread_t queue_thread;
	static void *incomingMessagesThread(void *arg) ;
public:
	cCAN();
	cCAN(const char* port,const int& baut);
	virtual ~cCAN();
	double getVelocity() const;
	void setVelocity(const double& velocity);
	unsigned int hexStrToInt(char *str, int start, int len);
};

#endif /* CAN_H_ */
