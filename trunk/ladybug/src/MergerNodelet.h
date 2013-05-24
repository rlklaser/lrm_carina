/*
 * MergerNodelet.h
 *
 *  Created on: Aug 23, 2012
 *      Author: rlklaser
 */
#if false

#ifndef MERGERNODELET_H_
#define MERGERNODELET_H_

#include <nodelet/nodelet.h>

namespace LadyBugNodelet
{

class MergerNodelet : public nodelet::Nodelet
{
public:
	MergerNodelet();
	virtual ~MergerNodelet();
	virtual void onInit();
private:
	ros::Publisher pub_;
	ros::Subscriber sub_;
};

}
#endif /* MERGERNODELET_H_ */

#endif
