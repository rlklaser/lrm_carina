/*
 * CruiseControl.cpp
 *
 *  Created on: Sep 11, 2012
 *      Author: leandro
 */

#include <ros/ros.h>
#include <lrm_msgs/Throttle.h>
#include <lrm_msgs/Brake.h>
#include <lrm_msgs/Velocity.h>
#include <lrm_msgs/VehicleState.h>
#include <lrm_drivers/ControlVelocityBrake.h>

#define DEFAULT_MAX_VELOCITY 30
#define DEFAULT_BRAKE_INTENSITY 60
#define DEFAULT_RESET_BTN 0

#define  MAX_BRAKE_VALUE 40
#define MAX_ACCEL_VALUE 20
#define FREQUENCY 2.0

class CruiseControl {

protected:
	ros::NodeHandle nh, nh_priv;

	ros::Subscriber can_sub;
	ros::Subscriber cruiseVelocity_sub;

	ros::Publisher throttle_pub;
	ros::Publisher brake_pub;

	long int throttle_msg_counter;
	long int brake_msg_counter;
	double frequency;
	double switch_lag_throttle,switch_lag_brake;
	double brakeD;

	lrm_msgs::Velocity cruiseVelocity;
	lrm_msgs::Velocity lastVelocity;
	lrm_msgs::Throttle throttle;
	lrm_msgs::Brake brake;

	int max_accel;
	int max_brake;
	bool accelFlag;
	int topSpeed;

public:
	CruiseControl(ros::NodeHandle n);
	virtual ~CruiseControl();

	void cruiseVelocityCallback(const lrm_msgs::Velocity::ConstPtr& velocity);
	void canCallback(const lrm_msgs::VehicleState::ConstPtr& velocity);
	void inferenceControl(lrm_msgs::VehicleState state);
};

/* Default constructor.
 * @param n A ROS node handler, used to grab params and node settings from roscore.
 */
CruiseControl::CruiseControl(ros::NodeHandle n) :
		nh(n), nh_priv("~") {

	nh_priv.param("top_speed", topSpeed, DEFAULT_MAX_VELOCITY);
	nh_priv.param("max_accel", this->max_accel, MAX_ACCEL_VALUE);
	nh_priv.param("max_brake", this->max_brake, MAX_BRAKE_VALUE);
	nh_priv.param("frequency", this->frequency, FREQUENCY);
	nh_priv.param("switch_lag_brake", this->switch_lag_brake, 7.0);
	nh_priv.param("switch_lag_throttle", this->switch_lag_throttle, 7.0);

	//setting initial message values
	throttle_msg_counter = 0;
	brake_msg_counter = 0;
	throttle.value = 0;
	brake.value = 0;

	cruiseVelocity_sub = nh.subscribe<lrm_msgs::Velocity>("velocity_commands",10, &CruiseControl::cruiseVelocityCallback, this);
	can_sub = nh.subscribe<lrm_msgs::VehicleState>("vehicle_state", 10,	&CruiseControl::canCallback, this);

	brake_pub = nh.advertise<lrm_msgs::Brake>("brake_commands", 1);
	throttle_pub = nh.advertise<lrm_msgs::Throttle>("throttle_commands", 1);

	accelFlag = false;
}

/*
 * Default destructor.
 */
CruiseControl::~CruiseControl() {
}

void CruiseControl::cruiseVelocityCallback(const lrm_msgs::Velocity::ConstPtr& velocity) {
	this->cruiseVelocity.value = velocity->value;
	this->cruiseVelocity.header = velocity->header;
}

void CruiseControl::inferenceControl(lrm_msgs::VehicleState state) {
	double increaseAccel = 0;
	double increaseBrake = 0;
	lrm_msgs::Velocity tempVelocity;

	tempVelocity.value = state.velocity;
	tempVelocity.header = state.header;

	double erroV = tempVelocity.value - this->cruiseVelocity.value;
	double dV = tempVelocity.value - this->lastVelocity.value;

	ControlVelocityBrakeInferenceEngine(erroV, dV * FREQUENCY, &increaseAccel, &increaseBrake);

	if (((erroV < (-1 * switch_lag_throttle)) && (this->brakeD == 0)) || accelFlag) {
		//accel
		if ((this->throttle.value + increaseAccel) <= this->max_accel)
			this->throttle.value = this->throttle.value + increaseAccel;
		else
			this->throttle.value = this->max_accel;

		if ((this->throttle.value + increaseAccel) < 0.0)
			this->throttle.value = 0.0;

		this->brake.value = 0;
		accelFlag = true;

	}
	if (((erroV > switch_lag_brake) && (fabs(this->throttle.value) < 0.1)) || (accelFlag == false) ) {
		//brake
		if ((this->brakeD + increaseBrake) <= this->max_brake)
			this->brakeD = this->brakeD + increaseBrake;
		else
			this->brakeD = this->max_brake;

		if ((this->brakeD + increaseBrake) < 0)
			this->brakeD = 0;
		this->throttle.value = 0.0;
		this->brake.value = this->brakeD;
		accelFlag = false;

	}
	throttle.header.seq = throttle_msg_counter++;
	throttle.header.stamp = ros::Time::now();
	throttle.header.frame_id = "/throttle";
	throttle_pub.publish(throttle);

	brake.header.seq = brake_msg_counter++;
	brake.header.stamp = ros::Time::now();
	brake.header.frame_id = "/brake";
	brake_pub.publish(brake);

}

void CruiseControl::canCallback(const lrm_msgs::VehicleState::ConstPtr& state) {

	lrm_msgs::VehicleState tempState;
	tempState.header = state->header;
	tempState.velocity = state->velocity;

	this->inferenceControl(tempState);

	this->lastVelocity.value = state->velocity;
	this->lastVelocity.header = state->header;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "cruise_control");
	ros::NodeHandle n;

	CruiseControl cruise_ctrl(n);

	ros::spin();
}
