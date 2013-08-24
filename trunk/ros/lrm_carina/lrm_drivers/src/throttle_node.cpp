/*
 * Throttle.cpp
 *
 *  Created on: 13/09/2011
 *      Author: leandro
 */

#include <pthread.h>
#include <queue>
#include <ros/ros.h>
#include "lrm_msgs/Throttle.h"
#include "lrm_drivers/arduino_throttle.h"
#include "lrm_drivers/vcmdas1_throttle.h"
#include <cstdlib>
#include <exception>
#include <stdexcept>

#include <signal.h>

#define ARDUINO_DEFAULT_BAUD 9600

class ThrottleNode {

protected:
	ros::NodeHandle nh, nh_priv;
	ros::Subscriber throttle_sub;
	Throttle * throttle;
public:
	ThrottleNode(ros::NodeHandle n) :
		nh(n), nh_priv("~") {
		int baud, min, max, base_addr, channel;
		std::string path;
		this->nh_priv.param("port", path, std::string("/dev/vcm_das_1")  );
		this->nh_priv.param("baud",baud , ARDUINO_DEFAULT_BAUD);
		this->nh_priv.param("min_value", min, 0);
		this->nh_priv.param("max_value", max, 40);
		this->nh_priv.param("base_addr", base_addr, 0x300);
		this->nh_priv.param("channel", channel, 1);
		if(path.compare(std::string("/dev/vcmdas1") ) != 0)
			this->throttle = new ArduinoThrottle(min, max,path, baud);
		else{
			try{
				this->throttle = new VcmDas1Throttle( max, path, base_addr, channel);
			}
			catch (std::bad_alloc& e){
				ROS_FATAL("Char Device VcmDas1 not found.");
				exit(-1);
			}
		}
		ROS_INFO("Configuring acceleration device at %s", path.c_str());
		throttle_sub = nh.subscribe<lrm_msgs::Throttle>("throttle_commands", 10, &ThrottleNode::throttleCallback, this);

	}

	~ThrottleNode() {
		delete this->throttle;
	}

	void throttleCallback(const lrm_msgs::Throttle::ConstPtr& p) {
		try {
			this->throttle->setAccel(p->value);
		}
		catch(std::bad_alloc& e ){
			exit(-1);
		}
		catch(std::exception& e ){
			ROS_ERROR("Error on set accel.");
		}
	}

};

ThrottleNode* node;

void sigsegv_handler(int sig) {
	ROS_INFO_STREAM("throttle node kill " << sig);
	delete node;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "throttle");
	ros::NodeHandle n;

	node = new ThrottleNode(n);

	signal(SIGTERM, &sigsegv_handler);
	signal(SIGINT, &sigsegv_handler);
	signal(SIGHUP, &sigsegv_handler);
	signal(SIGSEGV, &sigsegv_handler);

	ros::spin();

	return 0;
}
