/*
 * Vcmdas1.cpp
 *
 *  Created on: Feb 10, 2013
 *      Author: mac
 */

#include "lrm_drivers/vcmdas1_driver.h"

Vcmdas1::Vcmdas1() {
	fd = NULL;
	this->base_addr = 0;
	this->ain_range = 0;
	this->ain_offset = 0;
	this->ain_gain = 0;
	this->aout_range[0] = 0;
	this->aout_range[1] = 0;
	this->aout_offset[0] = 0;
	this->aout_offset[1] = 0;
	this->aout_gain[0] = 0;
	this->aout_gain[1] = 0;
	this->out_val = 0;
}

Vcmdas1::Vcmdas1(const string& charDevicePath, unsigned int base_addr,
		unsigned int ain_range, unsigned int aout_0_range,
		unsigned int aout_1_range) throw () {


	fd = open(charDevicePath.c_str(), O_RDWR);
	if (fd == -1) {
		throw std::bad_alloc();
		return;
	}
	this->base_addr = base_addr;
	this->ain_range = ain_range;
	this->ain_offset = 0;
	this->ain_gain = 0;
	this->aout_range[0] = aout_0_range;
	this->aout_range[1] = aout_1_range;
	this->aout_offset[0] = 0;
	this->aout_offset[1] = 0;
	this->aout_gain[0] = 0;
	this->aout_gain[1] = 0;
	this->out_val = 0;
	this->resetCard();

}

void Vcmdas1::resetCard() throw () {
	unsigned int data;
	/*---------------------------
	 * reset control register to
	 * power-on value
	 *---------------------------*/
	this->set_b(this->base_addr + CONTROL, 0);

	/*---------------------------
	 * reset analog input
	 *---------------------------*/
	this->set_b(this->base_addr, 0);
	data = this->analogIn(0);

	/*---------------------------
	 * reset all analog output
	 * channels to 0.000 volts
	 *---------------------------*/
	this->analogOut(0, 0);
	this->analogOut(1, 0);

	/*--------------------------------
	 *  reset parallel port
	 *--------------------------------*/
	OUT_SAVEW(DIGLO, 0);

	/*--------------------------------
	 *  Enable writes to the EEPROM
	 *--------------------------------*/
	this->enableWriteEEPROM();
}

unsigned int Vcmdas1::analogIn(unsigned int channel) throw () {

	unsigned int done;
	unsigned int timedout;
	int data;

	/*---------------------------
	 *  select channel and start the conversion
	 *---------------------------*/
	this->set_w(this->base_addr + SELECT, channel);
	this->set_b(this->base_addr + CONVERT, 01);

	/*---------------------------
	 *  wait for conversion with a timeout loop
	 *
	 *  Worst case, with Settling Time set at 10, a conversion
	 *  should be complete in 10+10=20 microseconds, we should
	 *  be safe allowing 200 microseconds.
	 *
	 *  Assuming each iteration of this polling
	 *  loop takes at least 20 clocks, and a
	 *  maximum clock rate of 100 Mhz, each iteration
	 *  takes at least .2 microseconds.  We need to
	 *  allow at least 103 microseconds, so we will
	 *  allow 1000 iterations, giving:
	 *    1000 * .2 = 200 microseconds.
	 *  This should be plenty, even an a speedy machine.
	 *  On the other hand, if it's a lowly 8088
	 *  running at 5 Mhz, and taking more like 80 clocks,
	 *  the timeout will occur in:
	 *    1000 * 80 * .2 = 1600 microseconds.
	 *---------------------------*/
	done = SSL_IN(base_addr + STATUS) & DONE_BIT;

	for (timedout = 8000; !done && timedout; timedout--) {
		done = SSL_IN(base_addr + STATUS) & DONE_BIT;
	}

	/*---------------------------
	 *  return 16-bit data
	 *---------------------------*/
	if (!done) {
		throw std::exception();
		return 0;
	} else {
		data = SSL_INW(base_addr + ADCLO);
	}
	return (data);
}

void Vcmdas1::analogOut(unsigned int channel, int code) throw () {

	register unsigned int port;
	int i;

	/*-------------------------
	 *  calculate the port
	 *  address
	 *-------------------------*/
	port = this->base_addr + SERCS;

	/* Enable the chip select for the D/A */
	this->set_b(port, VCMDAS1_CSDA);

	port = this->base_addr + SERDATA;

	/*-----------------------------------------------
	 * Shift vaule: 16 bits
	 D15 D14 D13 D12 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
	 --- --- --- --- --- --- -- -- -- -- -- -- -- -- -- --
	 1   B   A   NC  D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
	 *-----------------------------------------------*/

	// First bit always one
	this->set_b(port, 0x01);

	if (channel == 0)
		channel = 2;
	// Output channel B select
	this->set_b(port, channel);
	// Output channel A select
	this->set_b(port, (channel >> 1));
	// Output NC
	this->set_b(port, 0x00);

	for (i = 0; i < 12; i++) {
		this->set_b(port, ((code & 0x0800) >> 11));
		code = (code << 1);
	}

	port = this->base_addr + SERCS;
	// Disable all serial chip selects
	this->set_b(port, 0x00);
	// Do a read input, just for a time delay
	get_b(port);
	// Set D/A load to low
	this->set_b(port, VCMDAS1_CSDL);
	// Do a read input, just for a time delay
	get_b(port);
	// Disable all serial chip selects
	this->set_b(port, 0x00);

}

void Vcmdas1::enableWriteEEPROM() throw () {
	register unsigned int port;
	int i;

	/* calculate the port address */
	port = this->base_addr + SERCS;

	/* Enable the chip select for the EEPROM */
	this->set_b(port, VCMDAS1_CSEE);

	port = this->base_addr + SERDATA;

	/*-----------------------------------------------
	 * Shift vaule:  9 bits
	 SB OP OP A5 A4 A3 A2 A1 A0
	 -- -- -- -- -- -- -- -- --
	 1  0  0  1  1  X  X  X  X
	 *-----------------------------------------------*/

	/* Output start bit of 1 */
	this->set_b(port, 1);
	/* Output Opcode for EEPROM write enable */
	this->set_b(port, 0);
	this->set_b(port, 0);

	/* Output string of 1s */
	for (i = 0; i < 6; i++) {
		this->set_b(port, 1);
	}

	port = this->base_addr + SERCS;
	/* Disable all serial chip selects */
	this->set_b(port, 0x00);

}

unsigned int Vcmdas1::readEEPROM(unsigned int address) throw () {
	register unsigned int port, a;
	int i;
	unsigned int data;
	a = address;
	/* calculate the port address */
	port = this->base_addr + SERCS;

	/* Enable the chip select for the EEPROM */
	this->set_b(port, VCMDAS1_CSEE);

	port = this->base_addr + SERDATA;

	/*-----------------------------------------------
	 * Shift vaule:  9 bits
	 SB OP OP A5 A4 A3 A2 A1 A0
	 -- -- -- -- -- -- -- -- --
	 1  1  0  X  X  X  X  X  X
	 *-----------------------------------------------*/

	/* Output start bit of 1 */
	this->set_b(port, 1);
	/* Output Opcode for EEPROM read */
	this->set_b(port, 1);
	this->set_b(port, 0);

	/* Output address to read from */
	for (i = 0; i < 6; i++) {
		this->set_b(port, ((a & 0x0020) >> 5));
		a = (a << 1);
	}

	/* First data read is always 0 */
	/* Should a check could be done here to see if it is zero? */

	data = 0;
	for (i = 0; i < 16; i++) {
		data = (data << 1); // Make space for the next data bit
		this->set_b(port, 0);    // Dummy output to clock data
		data = (data | (0x01 & this->get_b(port)));
	}

	port = this->base_addr + SERCS;
	/* Disable all serial chip selects */
	this->set_b(port, 0x00);

	return data;

}

void Vcmdas1::writeEEPROM(unsigned int address, unsigned int data) throw () {
	unsigned int eecs_port, eedata_port, d, a;
	unsigned int i, done, timedout;
	d = data;
	a = address;
	/* Port address for the EEPROM chip select */
	eecs_port = base_addr + SERCS;

	/* Port address for the EEPROM serial data */
	eedata_port = base_addr + SERDATA;

	/* Raise the EEPROM chip select */
	this->set_b(eecs_port, VCMDAS1_CSEE);

	/*---------------------------------------------------------------------------------
	 * Shift value:  25 bits
	 SB OP OP A5 A4 A3 A2 A1 A0 D15 D14 D13 D12 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
	 1  0  1
	 *---------------------------------------------------------------------------------*/

	/* Output SB (Start Bit) */
	this->set_b(eedata_port, 1);

	/* Output OP OP (EEPROM WRITE) */
	this->set_b(eedata_port, 0);
	this->set_b(eedata_port, 1);

	/* Output A5-A0 */
	for (i = 0; i < 6; i++) {
		this->set_b(eedata_port, ((a & 0x0020) >> 5));
		a = (a << 1);
	}

	/* Output D15-D0 */
	for (i = 0; i < 16; i++) {
		this->set_b(eedata_port, ((d & 0x8000) >> 15));
		d = (d << 1);
	}

	/* Drop the EEPROM chip select briefly to reveal BUSY-READY status */this->set_b(
			eecs_port, 0x00);
	this->set_b(eecs_port, VCMDAS1_CSEE);

	/* Enter busy loop with timeout escape */
	done = this->get_b(eedata_port);
	done &= 0x01;
	for (timedout = 30000; !done && timedout; timedout--) {
		done = this->get_b(eedata_port);
		done &= 0x01;
	}

	/* ---------------------------------------------------------------
	 * Execution reaches this point when the write operation completes
	 * or timeout happens, whichever occurs first.
	 *----------------------------------------------------------------*/

	/* Drop the EEPROM chip select */
	this->set_b(eecs_port, 0x00);

	/* Return the appropriate error code  */
	if (!done) {
		throw std::exception();
	}

}

void Vcmdas1::analogOutMiliVolt(unsigned int channel,
		 int voltage) throw () {

	int outputval;
	int offset = aout_offset[channel];
	int voltRange, rawRange;
	rawRange = voltage > 0 ? 2047 - offset : 2048 + offset;

	/*---------------------------
	 *  convert to output value
	 *  with rounding
	 *---------------------------*/
	switch (aout_range[channel]) {

	/*--------------------------------
	 *  +/- 10 Volt Output Range:
	 *  Divide +/-10000 into
	 *  +/-2048
	 *--------------------------------*/
	case VCMDAS1_PM10:
		voltRange = 10000;
		break;

		/*--------------------------------
		 *  +/- 5 Volt Output
		 *  Divide +/-5000 into
		 *  +/-2048
		 *--------------------------------*/
	case VCMDAS1_PM5:
		voltRange = 5000;
		break;
	}
	outputval = (int) (((((long) voltage) * rawRange) + voltRange / 2) / voltRange);

	/*---------------------------
	 *  handle overflow
	 *---------------------------*/
	if (outputval > 2047)
		outputval = 2047;
	if (outputval < -2048)
		outputval = -2048;

	/*---------------------------
	 *  write the data and return
	 *  the error code
	 *---------------------------*/
	analogOut(channel, outputval);
}

unsigned int Vcmdas1::digitalIN(unsigned int modno) throw () {
	return ((get_w(base_addr + DIGLO) >> (15 - modno)) & 1);
}

void Vcmdas1::digitalOut(unsigned int modno, unsigned int val) throw () {
	unsigned int mask = 0x8000;
	unsigned int data = out_val;

	mask >>= modno;

	if (val)
		data |= mask;
	else
		data &= (~mask);

	OUT_SAVEW(DIGLO, data);

}

void Vcmdas1::ajustDigitalPot(unsigned int channel, unsigned int code) throw () {
	register unsigned int port;
	unsigned int i, c = code;
	// calculate the port address
	port = this->base_addr + SERCS;

	/* Enable the chip select for the DPot */
	this->set_b(port, VCMDAS1_CSDP);

	port = this->base_addr + SERDATA;

	/*-----------------------------------------------
	 * Shift vaule: 10 bits
	 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
	 -- -- -- -- -- -- -- -- -- --
	 A1 A0 D7 D6 D5 D4 D3 D2 D1 D0
	 *-----------------------------------------------*/

	// Output address 1
	this->set_b(port, (channel >> 1));
	// Output address 0
	this->set_b(port, (channel & 0x01));

	for (i = 0; i < 8; i++) {
		this->set_b(port, ((c & 0x0080) >> 7));
		c = (c << 1);
	}

	port = this->base_addr + SERCS;
	// Disable all serial chip selects.
	// Output 20000 times. This delay is to allow the dpot to settle.
	for (i = 0; i < 20000; i++) {
		this->set_b(port, 0x00);
	}

}

long Vcmdas1::analogInMicroVolt(unsigned int channel) throw () {

	float microvolts = 0;
	int data;

	/*---------------------------
	 *  read the channel
	 *---------------------------*/

	data = analogIn(channel);

	/*---------------------------
	 *  convert to microvolts
	 *  with rounding
	 *---------------------------*/
	switch (ain_range) {
	case VCMDAS1_PM10:
		/*---------------------------
		 *  +/-10V Input Range:
		 *  Multiply +/-32768 into
		 *  +/-10,000,000
		 *---------------------------*/

		microvolts = ((((float) data) * 10000000L) + 16384L) / 32768L;
		break;
	case VCMDAS1_PM5:
		/*---------------------------
		 *  +/-5V Input Range:
		 *  Multiply +/-32768 into
		 *  +/-5,000,000
		 *---------------------------*/

		microvolts = ((((float) data) * 5000000L) + 16384L) / 32768L;

		break;
	}

	return (long) microvolts;
}

void Vcmdas1::set_b(unsigned int port, unsigned int value) throw () {

	StructData pack;

	pack.address = port;
	pack.data = value;

	if (ioctl(fd, SET_B, &pack) != 0) {
		throw std::exception();
		return;
	}

}

void Vcmdas1::set_w(unsigned int port, unsigned int value) throw () {

	StructData pack;

	pack.address = port;
	pack.data = value;

	if (ioctl(fd, SET_W, &pack) != 0) {
		throw std::exception();
		return;
	}

}

unsigned int Vcmdas1::get_b(unsigned int port) throw () {

	StructData pack;

	pack.address = port;

	if (ioctl(fd, GET_B, &pack) != 0) {
		throw std::exception();
		return 0;
	}

	return pack.data;

}

unsigned int Vcmdas1::get_w(unsigned int port) throw () {

	StructData pack;

	pack.address = port;

	if (ioctl(fd, GET_W, &pack) != 0) {
		throw std::exception();
		return 0;
	}

	return pack.data;

}

Vcmdas1::~Vcmdas1() {
	this->resetCard();
	close(fd);
}

