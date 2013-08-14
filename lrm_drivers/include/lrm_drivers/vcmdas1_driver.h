/*
 * Vcmdas1.h
 *
 *  Created on: Feb 10, 2013
 *      Author: mac
 */

#ifndef VCMDAS1_H_
#define VCMDAS1_H_

#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <cassert>
#include <unistd.h>
#include <string>
#include <exception>
#include <stdexcept>
#include <cstdio>

using namespace std;

#define	MODULE_NAME "vcm_das_1"

typedef unsigned int vcm_reg32_t;

/* Operations vcm_das_1  */

typedef struct {
	vcm_reg32_t address;
	vcm_reg32_t data;

} StructData;

#define SET_B			_IOW('q',1 , StructData *)
#define SET_W 			_IOW('q',2 , StructData *)

#define GET_B			_IOWR('q',3 , StructData *)
#define GET_W 			_IOWR('q',4 , StructData *)

#define STATUS          0x00    /*  Read port */
#define CONTROL         0x00    /*  Write port */
#define SELECT          0x01    /*  Write port */
#define CONVERT         0x02    /*  Write port */
#define TDELAY          0x03    /*  Write port */
#define ADCLO           0x04    /*  Read only port */
#define ADCHI           0x05    /*  Read only port */
#define DIGLO           0x06    /*  Digital low - read/write */
#define DIGHI           0x07    /*  Digital high - read/write */
#define SERCS           0x08    /*  Serial Chip select - write */
#define SERSTAT         0x09    /*  Serial status - read port */
#define SERDATA         0x09    /*  Serial data send and clock - write port */

#define DONE_BIT        0x40    /* bit mask for A/D conversion complete */
#define BUSY_BIT        0x80    /* bit mask for A/D converion busy */
#define DA              0x01    /* bit mask for D/A chip select */
#define DPOT            0x02    /* bit mask for DPOT chip select */
#define EEPROM          0x04    /* bit mask for EEPROM chip select */

#define AINPUT_CG 	 	 0x7ffbL /* value read for gain calibration */
#define AOUTPUT_RB	 	 0x7fffL /* value read back for full scale output */

#define VCMDAS1_PM5                     0
#define VCMDAS1_PM10                    1

#define VCMDAS1_DATA_BUS_WIDTH_8        0
#define VCMDAS1_DATA_BUS_WIDTH_16       1

#define VCMDAS1_CSDA 0x01
#define VCMDAS1_CSDP 0x02
#define VCMDAS1_CSEE 0x04
#define VCMDAS1_CSDL 0x08

/* Digital Pot addresses */

#define VCMDAS1_DP_AD_OFFSET 	0x00
#define VCMDAS1_DP_AD_GAIN 	0x02
#define VCMDAS1_DP_DA0_GAIN 	0x01
#define VCMDAS1_DP_DA1_GAIN 	0x03

#define  SSL_IN	     get_b
#define  SSL_INW	 get_w
#define  SSL_OUT	 set_b
#define  SSL_OUTW	 set_w

void OUT_SAVEW(unsigned int p, unsigned int d);
#define OUT_SAVEW(p,d) (\
	 SSL_OUTW(this->base_addr + (p), (d)),\
	 this->out_val = (d))

class Vcmdas1 {
private:
	int fd;



	unsigned int base_addr;
	unsigned int ain_range;
	unsigned int ain_offset;
	unsigned int ain_gain;
	unsigned int aout_range[2];
	unsigned int aout_offset[2];
	unsigned int aout_gain[2];
	unsigned int out_val;

public:
	Vcmdas1();
	Vcmdas1(const string& path, unsigned int base_addr,
			unsigned int ain_range, unsigned int aout_0_range,
			unsigned int aout_1_range) throw ();
	void resetCard() throw ();
	long analogInMicroVolt(unsigned int channel)throw ();
	unsigned int analogIn(unsigned int channel) throw ();
	void analogOutMiliVolt(unsigned int channel,
			 int voltage) throw ();
	void analogOut(unsigned int channel,  int code) throw ();
	void ajustDigitalPot(unsigned int channel,
			unsigned int code) throw ();
	void enableWriteEEPROM()throw ();
	unsigned int readEEPROM(unsigned int address) throw ();
	void writeEEPROM(unsigned int address,
			unsigned int data) throw ();
	unsigned int digitalIN(unsigned int modno)throw ();
	void digitalOut(unsigned int modno, unsigned int val)throw ();

	unsigned int get_w(unsigned int port) throw ();
	unsigned int get_b(unsigned int port)throw () ;
	void set_w(unsigned int port, unsigned int value)throw ();
	void set_b(unsigned int port, unsigned int value)throw ();

	virtual ~Vcmdas1();
};

#endif /* VCMDAS1_H_ */
