/*
 * vcmdas1_throttle.h
 *
 *  Created on: Jan 27, 2013
 *      Author: Diego Gomes
 */

#ifndef VCMDAS1_THROTTLE_H_
#define VCMDAS1_THROTTLE_H_

#include "lrm_drivers/vcmdas1_driver.h"
#include "lrm_drivers/throttle.h"


#include <cassert>
#include <string>
#include <exception>
#include <cstdio>


#define MIN_VALUE_DAC 540 
#define MAX_VALUE_DAC 3768 // 3768 * 0.00122070313 = 4,60 Volts



class VcmDas1Throttle : public Throttle {
private:

protected:
	Vcmdas1 * card;
	unsigned int channel;

public:
	VcmDas1Throttle();
	VcmDas1Throttle(  const int&  max, const std::string& charDevicePath, const unsigned int& base_addr, const unsigned int& ch);
	virtual ~VcmDas1Throttle();
	void setAccel (const double& accel);
	const double& getAccel()const;
};
#endif /* VCMDAS1_THROTTLE_H_ */
