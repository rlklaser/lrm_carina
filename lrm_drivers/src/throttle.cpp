/*
 * vcmdas1_throttle.h
 *
 *  Created on: Jan 27, 2013
 *      Author: Diego Gomes
 */

#include "lrm_drivers/throttle.h"

Throttle::Throttle(const int& min, const int& max ) {
	this->min_value = min;
	this->max_value = max;
	this->currentAccel = -1.0;

}

Throttle::Throttle() {
	this->min_value = 0;
	this->max_value = 40;
	this->currentAccel = -1.0;
}

Throttle::~Throttle() {
}

const double& Throttle::getAccel() const{
	return this->currentAccel;
}
