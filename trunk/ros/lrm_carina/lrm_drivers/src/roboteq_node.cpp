/*
 * steering.cpp
 *
 *  Created on: 14/09/2011
 *      Author: leandro
 */

#include <math.h>
#include <ros/ros.h>
#include <lrm_drivers/roboteq.h>
#include <lrm_drivers/AX2550.h>
#include <lrm_drivers/HDC2450.h>
#include <lrm_msgs/Steering.h>
#include <lrm_msgs/Brake.h>
#include <lrm_msgs/Encoders.h>

#include <signal.h>

#define MAX_STEERING_ANGLE 30			/**< Maximum steering angle. */
#define ROBOTEQ_MODEL_AX2550 "AX2550"	/**< First supported RoboteQ model (that's the default choice). */
#define ROBOTEQ_MODEL_HDC2450 "HDC2450"	/**< Second supported RoboteQ model. */

#define ENC_WHEEL 0						/**< Wheel encoder channel at RoboteQ */
#define ENC_STEER 1						/**< Steering encoder channel at RoboteQ */

class RoboteqNode {
private:
	ros::NodeHandle nh, nh_priv;
	ros::Subscriber angle_subscriber; /**< The angle_subscriber subscribes this node at steering_commands topic. */
	ros::Subscriber brake_subscriber; /**< The brake_subscriber subscribes this node at brake_commands topic. */
	ros::Publisher encoder_publisher;
	ros::Publisher status_publisher;
	ros::Timer encoderTimer;

	std::string roboteqPath; /**< Holds the value of RoboteQ device path assign by ROS Param Server. */
	std::string roboteqModel; /**< Holds the value of RoboteQ's model assign by ROS Param Server. */
	RoboteQ *roboteq; /**< Represent the instance of RoboteQ's device */
	bool publish_encoder; /**< Is desired publish encoder values? */
	double encoder_freq; /**< If is desired publish encoder values, in which frequency it should be done? */
	double steering_angle; /**< Represents the value of current steering angle. */
	double brake_intensity; /**< Represents the value of current brake intensity. */

	double steering_angle_ratio;

	long int msg_counter;

public:
	RoboteqNode(ros::NodeHandle n) :
			nh(n), nh_priv("~") {

		nh_priv.param("port", roboteqPath, std::string("/dev/ttyUSB0"));
		nh_priv.param("model", roboteqModel, std::string(ROBOTEQ_MODEL_AX2550));
		nh_priv.param("publish_encoders", publish_encoder, true);
		nh_priv.param("encoders_rate", encoder_freq, 20.0);
		nh_priv.param("steering_angle_ratio", steering_angle_ratio, 4.1);

		angle_subscriber = nh.subscribe<lrm_msgs::Steering>("steering_commands", 1, &RoboteqNode::steeringCallback, this);
		brake_subscriber = nh.subscribe<lrm_msgs::Brake>("brake_commands", 1, &RoboteqNode::brakeCallback, this);

		if (roboteqModel.compare(ROBOTEQ_MODEL_AX2550) == 0)
			roboteq = new AX2550();
		else if (roboteqModel.compare(ROBOTEQ_MODEL_HDC2450) == 0)
			roboteq = new HDC2450();
		else {
			ROS_ERROR("RoboteQ model invalid or no defined, impossible to create a new instance of RoboteQ.");
			exit(-1);
		}

		roboteq->setSteeringEncoderRatio(steering_angle_ratio);
		roboteq->connect(this->roboteqPath.c_str());

		ROS_INFO("RoboteQ model %s was configured at serial port %s", this->roboteqModel.c_str(), this->roboteqPath.c_str());

		if (publish_encoder) {
			encoder_publisher = nh.advertise<lrm_msgs::Encoders>("encoders", 2, true); //latched
			encoderTimer = n.createTimer(ros::Duration(1.0/encoder_freq), &RoboteqNode::statePublisherCallback, this);
		}

		this->steering_angle = 0;
		this->brake_intensity = 0;
		this->msg_counter = 0;
	}

	~RoboteqNode() {
		delete roboteq;
	}

	/**
	 * This method grab all steering angle commands and dispatch the controller device to proceed the correspondent action.
	 * @param steer It represents a SteeringAngle message received from any control node.
	 */
	void steeringCallback(const lrm_msgs::Steering::ConstPtr& steer) {
		this->setAngle(steer->angle);
	}

	/**
	 * This method grab all brake intensity commands and dispatch the controller device to proceed the correspondent action.
	 * @param steer It represents a SteeringAngle message received from any control node.
	 */
	void brakeCallback(const lrm_msgs::Brake::ConstPtr& brake) {
		this->setBrake(brake->value);
	}

	/**
	 * It's a timer event routine to publish the encoder state
	 * @param none
	 */
	void statePublisherCallback(const ros::TimerEvent&) {
		lrm_msgs::Encoders encoder;

		encoder.header.frame_id = "/encoders";
		encoder.header.seq = msg_counter++;
		encoder.header.stamp = ros::Time::now();

		encoder.wheel.relative = roboteq->getEncoderValue(ENC_WHEEL, ENCODER_REL);
		encoder.steering.absolute = roboteq->getEncoderValue(ENC_STEER, ENCODER_ABS);

		encoder_publisher.publish(encoder);
	}

	/**
	 * Define the steering value
	 * @param angle correspond the wheel angle (in degrees)
	 */
	void setAngle(double angle) {
		this->steering_angle = angle;
		roboteq->steering(angle);
	}

	/**
	 * Define the braking intensity
	 * @param intensity correspond the braking intensity with 0 to 100 percent of range
	 */
	void setBrake(signed char intensity) {
		this->brake_intensity = intensity;
		roboteq->brake(intensity);
	}

};

RoboteqNode* node;

void sigsegv_handler(int sig) {
	ROS_INFO_STREAM("roboteq node kill " << sig);
	delete node;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "roboteq");
	ros::NodeHandle n;

	node = new RoboteqNode(n);

	signal(SIGTERM, &sigsegv_handler);
	signal(SIGINT, &sigsegv_handler);
	signal(SIGHUP, &sigsegv_handler);
	signal(SIGSEGV, &sigsegv_handler);

	ros::spin();

	return 0;
}
