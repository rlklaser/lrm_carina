/*
 * Serial.cpp
 *
 *  Created on: 22/11/2010
 *      Author: leandro
 */

#include "SerialComm.h"
#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <termios.h>

SerialComm::SerialComm() {
	this->fd = -1;
	portname[0] = '\0';
}

SerialComm::~SerialComm() {
	if (fd != -1)
		this->closePort();
}

/**
 * Sends data through serial port.
 * @param[in] buff contains the data to be sent.
 * @param[in] nbytes number of bytes to be read.
 * \return number of correct sent bytes.
 */
int SerialComm::send(void *buff, int nbytes) {

	int bytes = write(fd, buff, nbytes);

	if (bytes < 0) {
		std::cerr << "Failure to send data " << strerror(errno) << std::endl;
		return -1;
	}

	if (bytes != nbytes)
		std::cerr << "Data sent was truncated" << std::endl;

	return bytes;
}

/**
 * Receives data through serial port.
 * @param[out] buff contains the received data.
 * @param[in] nbytes number of bytes to be read.
 * \return number of retrieved bytes.
 */
int SerialComm::receive(void *buff, int nbytes) {

	int bytes = read(fd, buff, nbytes);
	if (bytes < 0) {
		std::cerr << "Failure to receive data " << strerror(errno) << std::endl;
		return -1;
	}

	return bytes;
}

/**
 * Open a serial channel communication, identify by port_name, using the current OS settings.
 * @param port_name identifies which port will be used (e.g. "/dev/ttyUSB0").
 * \return the file descriptor that identify the serial port or -1 if fail.
 */
int SerialComm::openPort(const char *port_name) {

	if (fd != -1)
		this->closePort();

	strcpy(this->portname, port_name);
	this->fd = open(port_name, O_RDWR, S_IRUSR | S_IWUSR);

	if (fd < 0) {
		std::cerr << "Unable to open serial port " << port_name << " erro: "
				<< strerror(errno) << std::endl;
		return -1;
	}

	return fd;
}

/**
 * Close the serial port.
 * Note: Test if is already open before close.
 */
void SerialComm::closePort() {
	if (fd == -1)
		return;
	close(fd);
	fd = -1;
}

/**
 * Configure the serial port with necessary parameters.
 * \return
 */
int SerialComm::configure() {

	struct termios settings;

	//clear struct for new port settings
	bzero(&settings, sizeof(struct termios));

	//B9600  = baudrate
	//CS8    = character size mask
	//CLOCAL = Ignore modem control lines
	//CREAD  = enable receiver
	//HUPCL  = the Linux driver will drop DTR control line when the serial port was closed, reseting the Arduino board
	settings.c_cflag = B9600 | CS8 | CLOCAL	| CREAD	| HUPCL;

	//IGNPAR = Ignore framing errors and parity errors
	//ICRNL  = Translate  carriage return to newline on input (unless IGNCR is set).
	settings.c_iflag = IGNPAR | ICRNL;
	settings.c_oflag = 0;

	//In canonical mode:
	// * Input  is  made  available  line by line.  An input line is available when one of the line delimiters is
	//   typed (NL, EOL, EOL2;  or  EOF  at the start of line).  Except in the case of EOF, the line delimiter is
	//   included in the buffer returned by read().
	// * Line editing is enabled (ERASE, KILL; and if the IEXTEN flag is  set: WERASE, REPRINT, LNEXT). A read()
	//   returns  at most one line of input; if the read() requested fewer bytes than are available in the current
	//   line of input, then only as many bytes as requested are read, and the remaining characters will be available
	//   for a future read().
	settings.c_lflag = ICANON;

	settings.c_cc[VINTR]   = 0;
	settings.c_cc[VQUIT]   = 0;
	settings.c_cc[VERASE]  = 0;
	settings.c_cc[VKILL]   = 0;
	settings.c_cc[VEOF]    = VEOF;
	settings.c_cc[VTIME]   = 0;
	settings.c_cc[VMIN]    = VMIN;
	settings.c_cc[VSWTC]   = 0;
	settings.c_cc[VSTART]  = 0;
	settings.c_cc[VSTOP]   = 0;
	settings.c_cc[VSUSP]   = 0;
	settings.c_cc[VEOL]    = 0;
	settings.c_cc[VREPRINT]= 0;
	settings.c_cc[VDISCARD]= 0;
	settings.c_cc[VWERASE] = 0;
	settings.c_cc[VLNEXT]  = 0;
	settings.c_cc[VEOL2]   = 0;

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

void SerialComm::setCtrlSignals(bool DTR, bool RTS) {

	int status;
	ioctl(this->fd, TIOCMGET, &status); /* get the serial port status */

	if (DTR)      /* set the DTR line */
		status &= ~TIOCM_DTR;
	else
		status |= TIOCM_DTR;

	if (RTS)      /* set the RTS line */
		status &= ~TIOCM_RTS;
	else
		status |= TIOCM_RTS;

	ioctl(this->fd, TIOCMSET, &status);
}

/**
 * Print all active serial port settings (cflags, iflags, oflags and control) at console.
 */
void SerialComm::getStatus() {
	struct termios settings;

	if (tcgetattr(fd, &settings) < 0)
		std::cerr << "Unable to get serial port attributes." << std::endl;

	char status[255];

	strcpy(status, "c_cflag [ ");
	strcat(status, (settings.c_cflag & CLOCAL) ? "CLOCAL " : "");
	strcat(status, (settings.c_cflag & HUPCL)  ? "HUPCL "  : "");
	strcat(status, (settings.c_cflag & CREAD)  ? "CREAD "  : "");
	strcat(status, (settings.c_cflag & CSTOPB) ? "CSTOPB " : "");
	strcat(status, (settings.c_cflag & PARENB) ? "PARENB " : "");
	strcat(status, (settings.c_cflag & PARODD) ? "PARODD " : "");
	strcat(status, (settings.c_cflag & CS5)    ? "CS5 "    : "");
	strcat(status, (settings.c_cflag & CS6)    ? "CS6 "    : "");
	strcat(status, (settings.c_cflag & CS7)    ? "CS7 "    : "");
	strcat(status, (settings.c_cflag & CS8)    ? "CS8 "    : "");
	strcat(status, (settings.c_cflag & CRTSCTS)? "CRTSCTS ": "");
	strcat(status, "]");
	std::cout << status << std::endl;

	strcpy(status, "c_iflag [ ");
	strcat(status, (settings.c_iflag & IGNBRK) ? "IGNBRK " : "");
	strcat(status, (settings.c_iflag & BRKINT) ? "BRKINT " : "");
	strcat(status, (settings.c_iflag & IGNPAR) ? "IGNPAR " : "");
	strcat(status, (settings.c_iflag & PARMRK) ? "PARMRK " : "");
	strcat(status, (settings.c_iflag & INPCK)  ? "INPCK "  : "");
	strcat(status, (settings.c_iflag & ISTRIP) ? "ISTRIP " : "");
	strcat(status, (settings.c_iflag & INLCR)  ? "INLCR "  : "");
	strcat(status, (settings.c_iflag & ICRNL)  ? "ICRNL "  : "");
	strcat(status, (settings.c_iflag & IUCLC)  ? "IUCLC "  : "");
	strcat(status, (settings.c_iflag & IXON)   ? "IXON "   : "");
	strcat(status, (settings.c_iflag & IXANY)  ? "IXANY "  : "");
	strcat(status, (settings.c_iflag & IXOFF)  ? "IXOFF "  : "");
	strcat(status, (settings.c_iflag & IMAXBEL)? "IMAXBEL ": "");
	strcat(status, "]");
	std::cout << status << std::endl;

	strcpy(status, "c_oflag [ ");
	strcat(status, (settings.c_oflag & OPOST) ? "OPOST " : "");
	strcat(status, (settings.c_oflag & OLCUC) ? "OLCUC " : "");
	strcat(status, (settings.c_oflag & ONLCR) ? "ONLCR " : "");
	strcat(status, (settings.c_oflag & OCRNL) ? "OCRNL " : "");
	strcat(status, (settings.c_oflag & ONOCR) ? "ONOCR " : "");
	strcat(status, (settings.c_oflag & ONLRET)? "ONLRET ": "");
	strcat(status, (settings.c_oflag & OFILL) ? "OFILL " : "");
	strcat(status, (settings.c_oflag & OFDEL) ? "OFDEL " : "");
	strcat(status, (settings.c_oflag & NLDLY) ? "NLDLY " : "");
	strcat(status, (settings.c_oflag & CRDLY) ? "CRDLY " : "");
	strcat(status, (settings.c_oflag & TABDLY)? "TABDLY ": "");
	strcat(status, (settings.c_oflag & BSDLY) ? "BSDLY " : "");
	strcat(status, (settings.c_oflag & VTDLY) ? "VTDLY " : "");
	strcat(status, (settings.c_oflag & FFDLY) ? "FFDLY " : "");
	strcat(status, "]");
	std::cout << status << std::endl;

	strcpy(status, "c_lflag [ ");
	strcat(status, (settings.c_lflag & ISIG)   ? "ISIG "   : "");
	strcat(status, (settings.c_lflag & ICANON) ? "ICANON " : "");
	strcat(status, (settings.c_lflag & XCASE)  ? "XCASE "  : "");
	strcat(status, (settings.c_lflag & ECHO)   ? "ECHO "   : "");
	strcat(status, (settings.c_lflag & ECHOE)  ? "ECHOE "  : "");
	strcat(status, (settings.c_lflag & ECHOK)  ? "ECHOK "  : "");
	strcat(status, (settings.c_lflag & ECHONL) ? "ECHONL " : "");
	strcat(status, (settings.c_lflag & ECHOCTL)? "ECHOCTL ": "");
	strcat(status, (settings.c_lflag & ECHOPRT)? "ECHOPRT ": "");
	strcat(status, (settings.c_lflag & ECHOKE) ? "ECHOKE " : "");
	strcat(status, (settings.c_lflag & FLUSHO) ? "FLUSHO " : "");
	strcat(status, (settings.c_lflag & NOFLSH) ? "NOFLSH " : "");
	strcat(status, (settings.c_lflag & TOSTOP) ? "TOSTOP " : "");
	strcat(status, (settings.c_lflag & PENDIN) ? "PENDIN " : "");
	strcat(status, (settings.c_lflag & IEXTEN) ? "IEXTEN " : "");
	strcat(status, "]");
	std::cout << status << std::endl;

	strcpy(status, "c_cc [ ");
	strcat(status, (settings.c_cc[VINTR])   ? "VINTR "   : "");
	strcat(status, (settings.c_cc[VQUIT])   ? "VQUIT "   : "");
	strcat(status, (settings.c_cc[VERASE])  ? "VERASE "  : "");
	strcat(status, (settings.c_cc[VKILL])   ? "VKILL "   : "");
	strcat(status, (settings.c_cc[VEOF])    ? "VEOF "    : "");
	strcat(status, (settings.c_cc[VMIN])    ? "VMIN "    : "");
	strcat(status, (settings.c_cc[VEOL])    ? "VEOL "    : "");
	strcat(status, (settings.c_cc[VTIME])   ? "VTIME "   : "");
	strcat(status, (settings.c_cc[VEOL2])   ? "VEOL2 "   : "");
	strcat(status, (settings.c_cc[VSTART])  ? "VSTART "  : "");
	strcat(status, (settings.c_cc[VSTOP])   ? "VSTOP "   : "");
	strcat(status, (settings.c_cc[VSUSP])   ? "VSUSP "   : "");
	strcat(status, (settings.c_cc[VLNEXT])  ? "VLNEXT "  : "");
	strcat(status, (settings.c_cc[VWERASE]) ? "VWERASE " : "");
	strcat(status, (settings.c_cc[VREPRINT])? "VREPRINT ": "");
	strcat(status, (settings.c_cc[VDISCARD])? "VDISCARD ": "");
	strcat(status, "]");
	std::cout << status << std::endl;
}
