/*
 * CAN.cpp
 *
 *  Created on: Feb 8, 2013
 *      Author: mac
 */

#include "lrm_drivers/cCAN.h"
#include <termios.h>
#include <string.h>
#include <unistd.h>

cCAN::cCAN(const char * port, const int& baud) {
	struct termios toptions;
	this->velocity = 0;

	fprintf(stderr, "init_serialport: opening port %s @ %d bps\n", port, baud);

	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		perror("init_serialport: Unable to open port ");
	}

	if (tcgetattr(fd, &toptions) < 0) {
		perror("init_serialport: Couldn't get term attributes");
	}
	speed_t brate = baud;
	cfsetispeed(&toptions, brate);
	cfsetospeed(&toptions, brate);

	// 8N1
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	// no flow control
	toptions.c_cflag &= ~CRTSCTS;

	toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	toptions.c_oflag &= ~OPOST; // make raw

	// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
	toptions.c_cc[VMIN] = 0;
	toptions.c_cc[VTIME] = 20;

	if (tcsetattr(fd, TCSANOW, &toptions) < 0) {
		perror("init_serialport: Couldn't set term attributes");
	}
	pthread_create(&queue_thread, NULL, &incomingMessagesThread, this);
	this->temp = 200;
}

cCAN::cCAN() {
	struct termios toptions;
	this->velocity = 0;

	fprintf(stderr, "init_serialport: opening port %s @ %d bps\n",
			CAN_DEFAULT_CHAR_DEVICE, CAN_DEFAULT_BAUD);

	fd = open(CAN_DEFAULT_CHAR_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		perror("init_serialport: Unable to open port ");
	}

	if (tcgetattr(fd, &toptions) < 0) {
		perror("init_serialport: Couldn't get term attributes");
	}
	speed_t brate = CAN_DEFAULT_BAUD;

	cfsetispeed(&toptions, brate);
	cfsetospeed(&toptions, brate);

	// 8N1
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	// no flow control
	toptions.c_cflag &= ~CRTSCTS;

	toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	toptions.c_oflag &= ~OPOST; // make raw

	// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
	toptions.c_cc[VMIN] = 0;
	toptions.c_cc[VTIME] = 20;

	if (tcsetattr(fd, TCSANOW, &toptions) < 0) {
		perror("init_serialport: Couldn't set term attributes");
	}
	pthread_create(&queue_thread, NULL, &incomingMessagesThread, this);
}

void* cCAN::incomingMessagesThread(void* arg) {
	char buf[SIZE_BUFF];
	int id;
	cCAN * can = (cCAN*) arg;
	while (true) {

		//memset( (void*)buf, 0, SIZE_BUFF );

		can->serialport_read(buf, '\n');

		id = can->hexStrToInt(buf, 1, 3);
		switch (id) {

		case ID_VELOCIDADE:
			can->setVelocity(can->calcVelocity(buf));
			break;
		}
	}
	return NULL;
}

int cCAN::serialport_read(char* buf, char until) {
    char b[1];
    int i=0;
    do {
    	 int n = read(fd, b, 1);  // read a char at a time
    	        if( n==-1 ) {
    	            usleep( 10 * 1000 ); // wait 10 msec try again
    	            continue;
    	        }
	if( b[0] == 't' ) i=0; // correction for overlapped messages
        buf[i] = b[0];
        i++;
        buf[i] = 0;
    } while( b[0] != until );
	buf[i] = 0;  // null terminate the string
	return 0;
}

double cCAN::getVelocity() const {
	return velocity;
}

double cCAN::calcVelocity(char* buf) {
	int num;
	num = this->hexStrToInt(buf, 6, 3);
	return (double) num * CAN_VELOCITY_AJUST;
}

void cCAN::setVelocity(const double& velocity) {
	this->velocity = velocity;
}

unsigned int  cCAN::hexStrToInt(char *str, int start, int len) {
	unsigned int result;
	char *tmp;
	tmp = (char*) malloc((len + 1) * sizeof(char));
	strncpy(tmp, str + start, len);
	tmp[len] = '\0';
	sscanf(tmp, "%x", &result);
	return result;
}

cCAN::~cCAN() {
	close(fd);
}
