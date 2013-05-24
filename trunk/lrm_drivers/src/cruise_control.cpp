/*
 * CruiseControl.cpp
 *
 *  Created on: Sep 11, 2012
 *      Author: leandro
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <atuacao/Throttle.h>
#include <atuacao/Brake.h>
#include <controle/Velocity.h>
#include <percepcao/Obstacles.h>
#include <atuacao/VehicleState.h>
#include <controle/ControlVelocityBrake.h>
//#include <sound_play/sound_play.h>

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
	ros::Subscriber obstacles_sub;
	ros::Subscriber joy_sub;

	ros::Publisher throttle_pub;
	ros::Publisher brake_pub;

	ros::Timer whatDogTimer;

	long int throttle_msg_counter;
	long int brake_msg_counter;
	double frequency;
	double switch_lag_throttle,switch_lag_brake;
	double brakeD;

	controle::Velocity cruiseVelocity;
	controle::Velocity lastVelocity;
	atuacao::Throttle throttle;
	atuacao::Brake brake;

	int max_accel;
	int max_brake;

	bool sinalWhatDog;
	bool accelFlag;
	bool isDisable;
	bool obstaclesDetected;

	int topSpeed;

	int linearAxis, velocity_btn_inc, velocity_btn_dec, reset_btn;

public:
	CruiseControl(ros::NodeHandle n);
	virtual ~CruiseControl();

	void cruiseVelocityCallback(const controle::Velocity::ConstPtr& velocity);
	void obstaclesCallback(const percepcao::Obstacles::ConstPtr& obstacles);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void whatDogCallback(const ros::TimerEvent& timer);
	void canCallback(const atuacao::VehicleState::ConstPtr& velocity);
	void inferenceControl(atuacao::VehicleState state);
};

/* Default constructor.
 * @param n A ROS node handler, used to grab params and node settings from roscore.
 */
CruiseControl::CruiseControl(ros::NodeHandle n) :
		nh(n), nh_priv("~") {

	nh_priv.param("top_speed", topSpeed, DEFAULT_MAX_VELOCITY);
	nh_priv.param("reset_btn", reset_btn, DEFAULT_RESET_BTN);

	nh_priv.param("max_accel", this->max_accel, MAX_ACCEL_VALUE);
	nh_priv.param("max_brake", this->max_brake, MAX_BRAKE_VALUE);
	nh_priv.param("frequency", this->frequency, FREQUENCY);
	nh_priv.param("switch_lag_brake", this->switch_lag_brake, 7.0);
	nh_priv.param("switch_lag_throttle", this->switch_lag_throttle, 7.0);

	if (!nh.getParam("/teleop_by_joy/brake_axis", linearAxis)) linearAxis = 1;
	if (!nh.getParam("/teleop_by_joy/velocity_increase_button", velocity_btn_inc)) velocity_btn_inc = 6;
	if (!nh.getParam("/teleop_by_joy/velocity_decrease_button", velocity_btn_dec)) velocity_btn_dec = 7;
	if (!nh.getParam("/teleop_by_joy/reset_button", reset_btn)) reset_btn = 3;

	isDisable = false;
	obstaclesDetected = false;

	//setting initial message values
	throttle_msg_counter = 0;
	brake_msg_counter = 0;
	throttle.value = 0;
	brake.value = 0;

	//index correction
	reset_btn--;

	cruiseVelocity_sub = nh.subscribe<controle::Velocity>("velocity_commands",10, &CruiseControl::cruiseVelocityCallback, this);
	obstacles_sub = nh.subscribe<percepcao::Obstacles>("obstacles", 1, &CruiseControl::obstaclesCallback, this);
	can_sub = nh.subscribe<atuacao::VehicleState>("vehicle_state", 10,	&CruiseControl::canCallback, this);
	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10,	&CruiseControl::joyCallback, this);

	brake_pub = nh.advertise<atuacao::Brake>("brake_commands", 1);
	throttle_pub = nh.advertise<atuacao::Throttle>("throttle_commands", 1);
	sinalWhatDog = false;
	accelFlag = false;
}

/*
 * Default destructor.
 */
CruiseControl::~CruiseControl() {
}

void CruiseControl::cruiseVelocityCallback(const controle::Velocity::ConstPtr& velocity) {
	if ((!isDisable) && (!obstaclesDetected)) {
		this->cruiseVelocity.value = velocity->value;
		this->cruiseVelocity.header = velocity->header;
	}else{
		this->cruiseVelocity.value = 0.0;
		this->cruiseVelocity.header = velocity->header;
	}
}

void CruiseControl::obstaclesCallback(const percepcao::Obstacles::ConstPtr& obstacles) {
	if (!isDisable) {
		this->throttle.value = 0.0;
		throttle.header.seq = throttle_msg_counter++;
		throttle.header.stamp = ros::Time::now();
		throttle.header.frame_id = "/throttle";
		throttle_pub.publish(throttle);
		this->brake.value = max_brake;
		brake.header.seq = brake_msg_counter++;
		brake.header.stamp = ros::Time::now();
		brake.header.frame_id = "/brake";
		brake_pub.publish(brake);

		obstaclesDetected = true;
		this->cruiseVelocity.value = 0.0;
	}
}

void CruiseControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	if ((joy->axes[linearAxis] != 0) && (joy->buttons[0])) {
		ROS_INFO("CruiseCtrl desabilitado");
		isDisable = true;
	}
	else if (joy->buttons[reset_btn]) {
		ROS_INFO("CruiseCtrl habilitado pelo RESET");
		isDisable = false;
		obstaclesDetected = false;
	}
	else {
		isDisable = false;
		ROS_INFO("CruiseCtrl habilitado");
	}
}

void CruiseControl::whatDogCallback(const ros::TimerEvent& timer) {
	if (sinalWhatDog && !isDisable) {
		throttle.header.seq = throttle_msg_counter++;
		throttle.header.stamp = ros::Time::now();
		throttle.header.frame_id = "/throttle";
		throttle.value = 0.0;
		throttle_pub.publish(throttle);
	}
	sinalWhatDog = true;
}

void CruiseControl::inferenceControl(atuacao::VehicleState state) {
	double increaseAccel = 0;
	double increaseBrake = 0;
	controle::Velocity tempVelocity;

	tempVelocity.value = state.velocity;
	tempVelocity.header = state.header;

	double erroV = tempVelocity.value - this->cruiseVelocity.value;
	double dV = tempVelocity.value - this->lastVelocity.value;

	sinalWhatDog = false;

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

void CruiseControl::canCallback(const atuacao::VehicleState::ConstPtr& state) {

	atuacao::VehicleState tempState;
	tempState.header = state->header;
	tempState.velocity = state->velocity;

	if (!isDisable && !obstaclesDetected) {
		this->inferenceControl(tempState);
	}
	this->lastVelocity.value = state->velocity;
	this->lastVelocity.header = state->header;
	sinalWhatDog = false;
}

/**
 *
 */
int main(int argc, char** argv) {
	ros::init(argc, argv, "cruise_control");
	ros::NodeHandle n;

//	sound_play::SoundClient sc;
//	sleep(2);
//	sc.playWave("/home/lrm/sounds/sistemas.ogg");

	CruiseControl cruise_ctrl(n);

	ros::spin();
}
