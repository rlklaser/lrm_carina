/*
 * Serial.h
 *
 *  Created on: 22/11/2010
 *      Author: leandro
 */

#ifndef SERIALCOMM_H_
#define SERIALCOMM_H_

class SerialComm {
	int fd; 			/**< file descriptor used to identify the serial port */
	char portname[80];	/**< holds serial port name */
public:
	SerialComm();
	virtual ~SerialComm();
	int openPort(const char *port_name);
	void closePort();
	int send(void *buff, int nbytes);
	int receive(void *buff, int nbytes);
	int configure();
	void getStatus();
	void setCtrlSignals(bool DTR, bool RTS);
};

#endif /* SERIAL_H_ */
