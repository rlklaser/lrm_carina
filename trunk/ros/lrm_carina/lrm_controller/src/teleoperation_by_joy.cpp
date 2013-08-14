/*
 * teleoperationbyjoy.cpp
 *
 *  Created on: 12/09/2011
 *      Author: leandro
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <lrm_msgs/Steering.h>
#include <lrm_msgs/Throttle.h>
#include <lrm_msgs/Brake.h>
#include <lrm_msgs/Velocity.h>

#define AXIS_FORWARD_BACKWARD 1
#define AXIS_LEFT_RIGHT 2

/**
 *
 */
class TeleopByJoy {

private:
	int linearAxis, angularAxis;
	int velocity_btn_inc, velocity_btn_dec, velocity_gain, reset_btn;
	double brake_scale, velocity_scale, steering_scale;

	lrm_msgs::Steering steer;
	lrm_msgs::Throttle throttle;
	lrm_msgs::Brake brake;
	lrm_msgs::Velocity velocity;

	long int steer_msg_counter;
	long int velocity_msg_counter;
	long int throttle_msg_counter;
	long int brake_msg_counter;

	ros::Publisher steering_pub;
	ros::Publisher throttle_pub;
	ros::Publisher brake_pub;
	ros::Publisher velocity_pub;
	ros::Subscriber joy_sub;
	ros::NodeHandle nh, nh_priv;

public:
	TeleopByJoy(ros::NodeHandle n);
	~TeleopByJoy();
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

};

TeleopByJoy::TeleopByJoy(ros::NodeHandle n) :
	nh(n), nh_priv("~") {

	nh_priv.param("brake_axis", linearAxis, AXIS_FORWARD_BACKWARD);
	nh_priv.param("brake_scale", brake_scale, 1.0);

	nh_priv.param("steering_axis", angularAxis, AXIS_LEFT_RIGHT);
	nh_priv.param("steering_scale", steering_scale, 1.0);

	nh_priv.param("velocity_increase_button", velocity_btn_inc, 6);
	nh_priv.param("velocity_decrease_button", velocity_btn_dec, 7);
	nh_priv.param("velocity_scale", velocity_scale, 1.0);

	nh_priv.param("reset_button", reset_btn, 3);

	//setting initial message values
	steer_msg_counter = 0;	velocity_msg_counter = 0;	brake_msg_counter = 0;	throttle_msg_counter = 0;
	steer.angle = 0;		velocity.value = 0;			brake.value = 0;		throttle.value = 0;

	//create the correct buttons index
	velocity_btn_inc--;
	velocity_btn_dec--;
	reset_btn--;

	velocity_gain = 100/velocity_scale;

	steering_pub = nh.advertise<lrm_msgs::Steering> ("steering_commands",1);
	throttle_pub = nh.advertise<lrm_msgs::Throttle> ("throttle_commands",1);
	brake_pub    = nh.advertise<lrm_msgs::Brake> ("brake_commands",1);
	velocity_pub = nh.advertise<lrm_msgs::Velocity> ("velocity_commands",1);

	joy_sub = nh.subscribe<sensor_msgs::Joy> ("joy", 10, &TeleopByJoy::joyCallback, this);
}

TeleopByJoy::~TeleopByJoy() {
}

void TeleopByJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	/*
	 * Steering commands
	 * Default: horizontal axis (left and right)
	 */
	if (joy->axes[angularAxis] != 0) {
		steer.header.seq = steer_msg_counter++;
		steer.header.stamp = ros::Time::now();
		steer.header.frame_id = "/base_link";
		steer.angle = steering_scale * joy->axes[angularAxis];
		steering_pub.publish(steer);
	}

	/*
	 * Velocity commands
	 * Default: buttons 6 and 7 (at base)
	 */
	if (joy->buttons[velocity_btn_inc]) {
		//Release brake pedal before throttle
		if (brake.value != 0) {
			brake.header.seq = brake_msg_counter++;
			brake.header.stamp = ros::Time::now();
			brake.header.frame_id = "/brake";
			brake.value = 0;
			brake_pub.publish(brake);
		}
		velocity.header.seq = velocity_msg_counter++;
		velocity.header.stamp = ros::Time::now();
		velocity.header.frame_id = "/velocity";
		if ((velocity.value + velocity_gain) <= 100)
			velocity.value = (uint8_t) velocity.value + velocity_gain;
		else
			velocity.value = 100;
		velocity_pub.publish(velocity);
	}
	if (joy->buttons[velocity_btn_dec]) {
		//Release brake pedal before throttle
		if (brake.value != 0) {
			brake.header.seq = brake_msg_counter++;
			brake.header.stamp = ros::Time::now();
			brake.header.frame_id = "/brake";
			brake.value = 0;
			brake_pub.publish(brake);
		}
		velocity.header.seq = velocity_msg_counter++;
		velocity.header.stamp = ros::Time::now();
		velocity.header.frame_id = "/velocity";
		if ((velocity.value - velocity_gain) >= 0)
			velocity.value = (uint8_t) velocity.value - velocity_gain;
		else
			velocity.value = 0.0;
		velocity_pub.publish(velocity);
	}

	/*
	 * Brake commands
	 * Default: vertical axis (forward and backward)
	 */
	if ((joy->axes[linearAxis] != 0) && (joy->buttons[0])) {
		if (joy->axes[linearAxis] <= 0) {
			//Release throttle pedal before brake
			if (velocity.value != 0) {
				velocity.header.seq = velocity_msg_counter++;
				velocity.header.stamp = ros::Time::now();
				velocity.header.frame_id = "/velocity";
				velocity.value = 0.0;
				velocity_pub.publish(velocity);

				throttle.header.seq = throttle_msg_counter++;
				throttle.header.stamp = ros::Time::now();
				throttle.header.frame_id = "/throttle";
				throttle.value = 0.0;
				throttle_pub.publish(throttle);
			}
			brake.header.seq = brake_msg_counter++;
			brake.header.stamp = ros::Time::now();
			brake.header.frame_id = "/brake";
			brake.value = -(uint8_t)(brake_scale * joy->axes[linearAxis]);
			brake_pub.publish(brake);
		}
	}

	/*
	 * Reset all operations
	 * Default: button 3
	 */
	if (joy->buttons[reset_btn]) {
		steer.header.seq = steer_msg_counter++;
		steer.header.stamp = ros::Time::now();
		steer.header.frame_id = "/steering";
		steer.angle = 0;
		steering_pub.publish(steer);

		velocity.header.seq = velocity_msg_counter++;
		velocity.header.stamp = ros::Time::now();
		velocity.header.frame_id = "/velocity";
		velocity.value = 0.0;
		velocity_pub.publish(velocity);

		throttle.header.seq = throttle_msg_counter++;
		throttle.header.stamp = ros::Time::now();
		throttle.header.frame_id = "/throttle";
		throttle.value = 0.0;
		throttle_pub.publish(throttle);

		brake.header.seq = brake_msg_counter++;
		brake.header.stamp = ros::Time::now();
		brake.header.frame_id = "/brake";
		brake.value = 0;
		brake_pub.publish(brake);
	}

}

/**
 *
 */
int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_by_joy");
	ros::NodeHandle n;

	TeleopByJoy teleop(n);

	ros::Rate(5);
	ros::spin();
}
