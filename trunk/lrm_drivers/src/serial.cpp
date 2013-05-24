/*
 * serial.cpp
 *
 *  Created on: 26/09/2011
 *      Author: leandro
 */

#include "lrm_drivers/serial.h"
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <cstdio>
#include <cstddef>
#include <ros/ros.h>

/**
 * Default constructor.
 * Create an empty instance, that has no file descriptor and none serial path was defined yet.
 */
Serial::Serial() {
	fd = -1;
	portname[0] = '\0';
}

/**
 * Destructor.
 * Restore all previous serial port settings and release resources.
 */
Serial::~Serial() {
	if (fd != -1) {
		if (tcsetattr(fd, TCSANOW, &oldsettings) < 0) {
			throw SerialException("Unable to recover serial port attributes");
		}
		if (tcflush(fd, TCIOFLUSH) != 0) {
			throw SerialException("Failure when flush port settings");
		}
		closePort();
	}
}

/**
 * Open the serial port identified by port_name.
 * @param port_name Serial port path (e.g. /dev/ttyUSB0)
 * @return The value of file descriptor in success case; or -1 if open port was failed.
 */
int Serial::openPort(const char *port_name) {
	if (fd != -1)
		closePort();

	strcpy(portname, port_name);
	fd = open(port_name, O_RDWR | O_NOCTTY );

	if (fd < 0) {
		std::string msg;
		msg.assign("Unable to open serial port ");
		msg.append(port_name);
		msg.append(" erro: ");
		msg.append(strerror(errno));

		throw SerialException(msg);
	}

	return fd;
}

/**
 * Configure the main serial port settings, like baud rate and operation mode.
 * @param baud Sets the transfer speed (baud rate). The accepted values are: 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200.
 * @param databits Value of data size, that can be: 5, 6, 7 or 8 bits.
 * @param isCanonical Sets the use or not of canonical transfer mode.
 * @param parity Parity checks. Uses 0 for none (parity disable); 1 for odd and 2 for even.
 * @param startbit Uses start bit frame checking.
 * @param stopbit Uses stop bit frame checking.
 * @param vmin Define how many character need to be read until to unblock a receive command (primitive).
 * @param vtime If vmin is zero, define a time out to receive a message and unblock the command primitive.
 * @return Return 1 in success case or -1 otherwise.
 */
int Serial::configure(int baud, int databits, bool isCanonical, int parity, int startbit, int stopbit, int vmin, int vtime) {
	int BAUDRATE;

	// save current port settings
	tcgetattr(fd,&oldsettings);

	// clear struct for new port settings
	bzero(&settings, sizeof(struct termios));

	switch (baud) {
	case 300:
		BAUDRATE = B300;
		break;
	case 1200:
		BAUDRATE = B1200;
		break;
	case 2400:
		BAUDRATE = B2400;
		break;
	case 4800:
		BAUDRATE = B4800;
		break;
	case 9600:
		BAUDRATE = B9600;
		break;
	case 19200:
		BAUDRATE = B19200;
		break;
	case 38400:
		BAUDRATE = B38400;
		break;
	case 57600:
		BAUDRATE = B57600;
		break;
	case 115200:
		BAUDRATE = B115200;
		break;
	default:
		BAUDRATE = B9600;
		break;
	}

	//CLOCAL : local connection, no modem control
	//CREAD  : enable receiving characters
	settings.c_cflag = BAUDRATE | CLOCAL | CREAD;

	//CRTSCTS : output hardware flow control (only used if the cable has
	//		    all necessary lines. See sect. 7 of Serial-HOWTO)
	settings.c_cflag &= ~CRTSCTS;

	//Character size mask. Values are CS5, CS6, CS7, or CS8.
	//When setting the character size, remember to mask using CSIZE first;
	settings.c_cflag &= ~CSIZE;
	switch (databits) {
	case 5:
		settings.c_cflag |= CS5;
		break;
	case 6:
		settings.c_cflag |= CS6;
		break;
	case 7:
		settings.c_cflag |= CS7;
		break;
	case 8:
		settings.c_cflag |= CS8;
		break;
	}

	// If you need to generate even parity, then set PARENB and clear PARODD;
	// if you need to generate odd parity then set both PARENB and PARODD.
	// If you don't want parity at all, then make sure PARENB is clear.
	if (parity == 0) {
		// ignore parity
		settings.c_cflag |= IGNPAR;
	}
	else {
		// use parity
		settings.c_cflag |= PARENB;
		settings.c_cflag &= (parity == 1) ? PARODD : ~PARODD;
	}

	// Set two stop bits, rather than one.
	settings.c_cflag &= (stopbit == 1) ? ~CSTOPB : CSTOPB;

	// IGNPAR : Ignore framing errors and parity errors.
	// ICRNL  : Translate carriage return to newline on input (unless IGNCR is set).
	// IGNBRK : If this bit is set, break conditions are ignored.
	//			A break condition is defined in the context of asynchronous serial data transmission as a series of zero-value bits longer than a single byte.
	// BRKINT : If this bit is set and IGNBRK is not set, a break condition clears the terminal input and output queues and raises a SIGINT signal for the
	//			foreground process group associated with the terminal. If neither BRKINT nor IGNBRK are set, a break condition is passed to the application
	//			as a single '\0' character if PARMRK is not set, or otherwise as a three-character sequence '\377', '\0', '\0'.
	// INLCR  : If this bit is set, newline characters ('\n') received as input are passed to the application as carriage return characters ('\r').
	// IMAXBEL: If this bit is set, then filling up the terminal input buffer sends a BEL character (code 007) to the terminal to ring the bell.
	settings.c_iflag = 0;
	settings.c_iflag = IGNBRK | ICRNL;

	// Raw output.
	settings.c_oflag = 0;

	//ICANON : enable canonical input
	//	  	   disable all echo functionality, and don't send signals to calling program
	//In canonical mode:
	// * Input  is  made  available  line by line.  An input line is available when one of the line delimiters is
	//   typed (NL, EOL, EOL2;  or  EOF  at the start of line).  Except in the case of EOF, the line delimiter is
	//   included in the buffer returned by read().
	// * Line editing is enabled (ERASE, KILL; and if the IEXTEN flag is  set: WERASE, REPRINT, LNEXT). A read()
	//   returns  at most one line of input; if the read() requested fewer bytes than are available in the current
	//   line of input, then only as many bytes as requested are read, and the remaining characters will be available
	//   for a future read().
	settings.c_lflag &= (isCanonical) ? ICANON : ~(ICANON | ECHO | ECHOE | ISIG);

	//initialize some control characters
	//default values can be found in /usr/include/termios.h, and are given in the comments
	settings.c_cc[VTIME] = vtime;	/* inter-character timer unused */
	settings.c_cc[VMIN]  = vmin;	/* blocking read until 1 character arrives */

	tcflush(fd, TCIFLUSH);

	if (tcsetattr(fd, TCSANOW, &settings) < 0) {
		std::cerr << "Unable to set serial port attributes" << std::endl;
		return -1;
	}

	// Make sure queues are empty before we begin
	if (tcflush(fd, TCIOFLUSH) != 0) {
		std::cerr << "Failure when flush port settings" << std::endl;
		return -1;
	}

	return 1;
}

/**
 * Check the status of connection.
 * @return If the serial port is connected or not.
 */
bool Serial::isConnected() {
	return (fd == -1) ? false : true;
}

/**
 * Close then serial port.
 * Check if is open before closing.
 */
void Serial::closePort() {
	if (fd != -1) {
		close(fd);
		fd = -1;
	}
}

/**
 * Send a message through serial port.
 * @param buff Base address of data.
 * @param nbytes Message length (in bytes).
 * @return How many bytes are transmitted.
 */
int Serial::send(void *buff, int nbytes) {
	if (isConnected()) {
		int ret = write(fd, buff, nbytes);
		tcdrain(fd);

//		char buff_out[256], *cbuff = (char*)buff;
//		_MAKE_PRINTABLE_CMD_(buff_out, cbuff, nbytes)
//		ROS_DEBUG_NAMED("Serial","Serial::send(%s,%d):%d", buff_out, nbytes, ret);

		return ret;
	}
	else
		throw SerialException("The serial port is not connected.");
}

/**
 * Receive a message through serial port.
 * @param buff Base address of get data.
 * @param nbytes Maximum message length (in bytes).
 * @return How many bytes are received.
 */
int Serial::receive(void *buff, int nbytes) {
	if (isConnected()) {
		int ret = read(fd, buff, nbytes);
				
//		char buff_out[256], *cbuff = (char*)buff;
//
//		_MAKE_PRINTABLE_CMD_(buff_out, cbuff, ret)
//		ROS_DEBUG_NAMED("Serial","Serial::receive(\'%x\',%d) : %d\n", buff_out, nbytes, ret);

		return ret;
	}
	else
		throw SerialException("The serial port is not connected.");
}

/**
 * Auxiliary method to verify the current serial port settings
 */
void Serial::printCurrentSettings() {
	char status[255];

	struct termios current_settings;

	// save current port settings
	tcgetattr(fd,&current_settings);

	strcpy(status, "Current Settings\n\tc_iflag [ ");
	strcat(status, (current_settings.c_iflag & IGNBRK)  ? "IGNBRK " : "");
	strcat(status, (current_settings.c_iflag & BRKINT)  ? "BRKINT " : "");
	strcat(status, (current_settings.c_iflag & IGNPAR)  ? "IGNPAR " : "");
	strcat(status, (current_settings.c_iflag & PARMRK)  ? "PARMRK " : "");
	strcat(status, (current_settings.c_iflag & INPCK)   ? "INPCK " : "");
	strcat(status, (current_settings.c_iflag & ISTRIP)  ? "ISTRIP " : "");
	strcat(status, (current_settings.c_iflag & INLCR)   ? "INLCR " : "");
	strcat(status, (current_settings.c_iflag & ICRNL)   ? "ICRNL " : "");
	strcat(status, (current_settings.c_iflag & IUCLC)   ? "IUCLC " : "");
	strcat(status, (current_settings.c_iflag & IXON)    ? "IXON " : "");
	strcat(status, (current_settings.c_iflag & IXANY)   ? "IXANY " : "");
	strcat(status, (current_settings.c_iflag & IXOFF)   ? "IXOFF " : "");
	strcat(status, (current_settings.c_iflag & IMAXBEL) ? "IMAXBEL " : "");
	strcat(status, "]");
	std::cout << status << std::endl;

	strcpy(status, "\tc_oflag [ ");
	strcat(status, (current_settings.c_oflag & OPOST)  ? "OPOST " : "");
	strcat(status, (current_settings.c_oflag & OLCUC)  ? "OLCUC " : "");
	strcat(status, (current_settings.c_oflag & ONLCR)  ? "ONLCR " : "");
	strcat(status, (current_settings.c_oflag & OCRNL)  ? "OCRNL " : "");
	strcat(status, (current_settings.c_oflag & ONOCR)  ? "ONOCR " : "");
	strcat(status, (current_settings.c_oflag & ONLRET) ? "ONLRET " : "");
	strcat(status, (current_settings.c_oflag & OFILL)  ? "OFILL " : "");
	strcat(status, (current_settings.c_oflag & OFDEL)  ? "OFDEL " : "");
	strcat(status, (current_settings.c_oflag & NLDLY)  ? "NLDLY " : "");
	strcat(status, (current_settings.c_oflag & CRDLY)  ? "CRDLY " : "");
	strcat(status, (current_settings.c_oflag & TABDLY) ? "TABDLY " : "");
	strcat(status, (current_settings.c_oflag & BSDLY)  ? "BSDLY " : "");
	strcat(status, (current_settings.c_oflag & VTDLY)  ? "VTDLY " : "");
	strcat(status, (current_settings.c_oflag & FFDLY)  ? "FFDLY " : "");
	strcat(status, "]");
	std::cout << status << std::endl;

	strcpy(status, "\tc_cflag [ ");
	strcat(status, (current_settings.c_cflag & CLOCAL)  ? "CLOCAL " : "");
	strcat(status, (current_settings.c_cflag & HUPCL)   ? "HUPCL " : "");
	strcat(status, (current_settings.c_cflag & CREAD)   ? "CREAD " : "");
	strcat(status, (current_settings.c_cflag & CSTOPB)  ? "CSTOPB " : "");
	strcat(status, (current_settings.c_cflag & PARENB)  ? "PARENB " : "");
	strcat(status, (current_settings.c_cflag & PARODD)  ? "PARODD " : "");
	strcat(status, (current_settings.c_cflag & CS5)     ? "CS5 " : "");
	strcat(status, (current_settings.c_cflag & CS6)     ? "CS6 " : "");
	strcat(status, (current_settings.c_cflag & CS7)     ? "CS7 " : "");
	strcat(status, (current_settings.c_cflag & CS8)     ? "CS8 " : "");
	strcat(status, (current_settings.c_cflag & CRTSCTS) ? "CRTSCTS " : "");
	strcat(status, "]");
	std::cout << status << std::endl;

	strcpy(status, "\tc_lflag [ ");
	strcat(status, (current_settings.c_lflag & ISIG)    ? "ISIG " : "");
	strcat(status, (current_settings.c_lflag & ICANON)  ? "ICANON " : "");
	strcat(status, (current_settings.c_lflag & XCASE)   ? "XCASE " : "");
	strcat(status, (current_settings.c_lflag & ECHO)    ? "ECHO " : "");
	strcat(status, (current_settings.c_lflag & ECHOE)   ? "ECHOE " : "");
	strcat(status, (current_settings.c_lflag & ECHOK)   ? "ECHOK " : "");
	strcat(status, (current_settings.c_lflag & ECHONL)  ? "ECHONL " : "");
	strcat(status, (current_settings.c_lflag & ECHOCTL) ? "ECHOCTL " : "");
	strcat(status, (current_settings.c_lflag & ECHOPRT) ? "ECHOPRT " : "");
	strcat(status, (current_settings.c_lflag & ECHOKE)  ? "ECHOKE " : "");
	strcat(status, (current_settings.c_lflag & FLUSHO)  ? "FLUSHO " : "");
	strcat(status, (current_settings.c_lflag & NOFLSH)  ? "NOFLSH " : "");
	strcat(status, (current_settings.c_lflag & TOSTOP)  ? "TOSTOP " : "");
	strcat(status, (current_settings.c_lflag & PENDIN)  ? "PENDIN " : "");
	strcat(status, (current_settings.c_lflag & IEXTEN)  ? "IEXTEN " : "");
	strcat(status, "]");
	std::cout << status << std::endl;

	strcpy(status, "\tc_cc [");
	for (int i = 0; i < 16; ++i) sprintf(status, "%s %d", status, current_settings.c_cc[i]);
	strcat(status, " ]");
	std::cout << status << std::endl;

	strcpy(status, "Program settings\n\tc_iflag [ ");
	strcat(status, (settings.c_iflag & IGNBRK)  ? "IGNBRK " : "");
	strcat(status, (settings.c_iflag & BRKINT)  ? "BRKINT " : "");
	strcat(status, (settings.c_iflag & IGNPAR)  ? "IGNPAR " : "");
	strcat(status, (settings.c_iflag & PARMRK)  ? "PARMRK " : "");
	strcat(status, (settings.c_iflag & INPCK)   ? "INPCK " : "");
	strcat(status, (settings.c_iflag & ISTRIP)  ? "ISTRIP " : "");
	strcat(status, (settings.c_iflag & INLCR)   ? "INLCR " : "");
	strcat(status, (settings.c_iflag & ICRNL)   ? "ICRNL " : "");
	strcat(status, (settings.c_iflag & IUCLC)   ? "IUCLC " : "");
	strcat(status, (settings.c_iflag & IXON)    ? "IXON " : "");
	strcat(status, (settings.c_iflag & IXANY)   ? "IXANY " : "");
	strcat(status, (settings.c_iflag & IXOFF)   ? "IXOFF " : "");
	strcat(status, (settings.c_iflag & IMAXBEL) ? "IMAXBEL " : "");
	strcat(status, "]");
	std::cout << status << std::endl;

	strcpy(status, "\tc_oflag [ ");
	strcat(status, (settings.c_oflag & OPOST)  ? "OPOST " : "");
	strcat(status, (settings.c_oflag & OLCUC)  ? "OLCUC " : "");
	strcat(status, (settings.c_oflag & ONLCR)  ? "ONLCR " : "");
	strcat(status, (settings.c_oflag & OCRNL)  ? "OCRNL " : "");
	strcat(status, (settings.c_oflag & ONOCR)  ? "ONOCR " : "");
	strcat(status, (settings.c_oflag & ONLRET) ? "ONLRET " : "");
	strcat(status, (settings.c_oflag & OFILL)  ? "OFILL " : "");
	strcat(status, (settings.c_oflag & OFDEL)  ? "OFDEL " : "");
	strcat(status, (settings.c_oflag & NLDLY)  ? "NLDLY " : "");
	strcat(status, (settings.c_oflag & CRDLY)  ? "CRDLY " : "");
	strcat(status, (settings.c_oflag & TABDLY) ? "TABDLY " : "");
	strcat(status, (settings.c_oflag & BSDLY)  ? "BSDLY " : "");
	strcat(status, (settings.c_oflag & VTDLY)  ? "VTDLY " : "");
	strcat(status, (settings.c_oflag & FFDLY)  ? "FFDLY " : "");
	strcat(status, "]");
	std::cout << status << std::endl;

	strcpy(status, "\tc_cflag [ ");
	strcat(status, (settings.c_cflag & CLOCAL)  ? "CLOCAL " : "");
	strcat(status, (settings.c_cflag & HUPCL)   ? "HUPCL " : "");
	strcat(status, (settings.c_cflag & CREAD)   ? "CREAD " : "");
	strcat(status, (settings.c_cflag & CSTOPB)  ? "CSTOPB " : "");
	strcat(status, (settings.c_cflag & PARENB)  ? "PARENB " : "");
	strcat(status, (settings.c_cflag & PARODD)  ? "PARODD " : "");
	strcat(status, (settings.c_cflag & CS5)     ? "CS5 " : "");
	strcat(status, (settings.c_cflag & CS6)     ? "CS6 " : "");
	strcat(status, (settings.c_cflag & CS7)     ? "CS7 " : "");
	strcat(status, (settings.c_cflag & CS8)     ? "CS8 " : "");
	strcat(status, (settings.c_cflag & CRTSCTS) ? "CRTSCTS " : "");
	strcat(status, "]");
	std::cout << status << std::endl;

	strcpy(status, "\tc_lflag [ ");
	strcat(status, (settings.c_lflag & ISIG)    ? "ISIG " : "");
	strcat(status, (settings.c_lflag & ICANON)  ? "ICANON " : "");
	strcat(status, (settings.c_lflag & XCASE)   ? "XCASE " : "");
	strcat(status, (settings.c_lflag & ECHO)    ? "ECHO " : "");
	strcat(status, (settings.c_lflag & ECHOE)   ? "ECHOE " : "");
	strcat(status, (settings.c_lflag & ECHOK)   ? "ECHOK " : "");
	strcat(status, (settings.c_lflag & ECHONL)  ? "ECHONL " : "");
	strcat(status, (settings.c_lflag & ECHOCTL) ? "ECHOCTL " : "");
	strcat(status, (settings.c_lflag & ECHOPRT) ? "ECHOPRT " : "");
	strcat(status, (settings.c_lflag & ECHOKE)  ? "ECHOKE " : "");
	strcat(status, (settings.c_lflag & FLUSHO)  ? "FLUSHO " : "");
	strcat(status, (settings.c_lflag & NOFLSH)  ? "NOFLSH " : "");
	strcat(status, (settings.c_lflag & TOSTOP)  ? "TOSTOP " : "");
	strcat(status, (settings.c_lflag & PENDIN)  ? "PENDIN " : "");
	strcat(status, (settings.c_lflag & IEXTEN)  ? "IEXTEN " : "");
	strcat(status, "]");
	std::cout << status << std::endl;

	strcpy(status, "\tc_cc [");
	for (int i = 0; i < 16; ++i) sprintf(status, "%s %d", status, settings.c_cc[i]);
	strcat(status, " ]");
	std::cout << status << std::endl;
}
