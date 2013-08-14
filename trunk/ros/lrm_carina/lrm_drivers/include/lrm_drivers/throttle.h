/*
 * vcmdas1_throttle.h
 *
 *  Created on: Jan 27, 2013
 *      Author: Diego Gomes
 */

#ifndef THROTTLE_H
#define THROTTLE_H

class Throttle {
private:
protected:
	int min_value;
	int max_value;
	double currentAccel;
public:
	Throttle();
	Throttle( const int& min, const int& max);
	virtual ~Throttle();
	virtual void setAccel(const double& ) = 0;
	virtual const double& getAccel() const;

};

#endif /** THROTTLE_H **/
