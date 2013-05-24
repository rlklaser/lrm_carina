/*
 * Copyright (c) 2012, Siddhant Ahuja (Sid)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Siddhant Ahuja (Sid) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Desc: Gazebo 1.x plugin for a Differential Drive Simulation Robot
 * Adapted from the Erratic and Turtlebot Robot plugin
 * Author: Siddhant Ahuja
 */

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

// ROS Specific
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//Make Key-codes to hex numbers
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

//Capital codes for keys. When shift is pressed. 
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

//Class for controlling the robot with a keyboard
class simBotKeyboardTeleopNode {
private:
	//Variables for walk/run velocities and walk/run yaw rates.
	double walk_vel_;
	double run_vel_;
	double yaw_rate_;
	double yaw_rate_run_;

	//Geometry message for Twist, internal to the code.
	geometry_msgs::Twist cmdvel_;

	//Initialize NodeHandle
	ros::NodeHandle n_;

	//Initialize ROS Publisher
	ros::Publisher pub_;

public:
	//Constructor for the class.
	simBotKeyboardTeleopNode() {
		//Start publishing Geometry-Twist messages on cmd_vel topic
		pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

		//Create an internal node handler
		ros::NodeHandle n_private("~");

		//Grab the values for walk/run velocities and yaw rates from the param server, or set some defaults.
		n_private.param("walk_vel", walk_vel_, 0.5);
		n_private.param("run_vel", run_vel_, 1.0);
		n_private.param("yaw_rate", yaw_rate_, 1.0);
		n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);
	}

	//De-constructor for the class.
	~simBotKeyboardTeleopNode() {
	}

	//Initialize keyboard loop. This is where all the input key processing occurs
	void keyboardLoop();

	//Function called when a SIGINT error is raised
	void quit(int sig);

	//If any other key besides W,A,S,D is pressed, then stop the robot.
	void stopRobot() {
		cmdvel_.linear.x = 0.0;
		cmdvel_.angular.z = 0.0;
		pub_.publish(cmdvel_);
	}
};

//Initialize some global variables
simBotKeyboardTeleopNode* simBot_keyboard_teleop;
int kfd = 0;
struct termios cooked, raw;
bool done;

//Main Loop
int main(int argc, char** argv) {
	//Initialize ROS
	ros::init(argc, argv, "simBotTeleopKeyboard", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);

	//Create an object of class simBotKeyboardTeleopNode
	simBotKeyboardTeleopNode simBot_keyboard_teleop;

	//Bind the Keyboard loop to the object
	boost::thread t = boost::thread(boost::bind(&simBotKeyboardTeleopNode::keyboardLoop, &simBot_keyboard_teleop));

	//Start spinning
	ros::spin();

	t.interrupt();
	t.join();

	//Stop the robot
	simBot_keyboard_teleop.stopRobot();

	//Set the new options for port
	tcsetattr(kfd, TCSANOW, &cooked);

	return (0);
}

void simBotKeyboardTeleopNode::keyboardLoop() {
	char c;
	double max_tv = walk_vel_;
	double max_rv = yaw_rate_;
	bool dirty = false;
	int speed = 0;
	int turn = 0;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~(ICANON | ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("Use WASD keys to control the robot");
	puts("Press Shift to move faster");

	struct pollfd ufd;
	ufd.fd = kfd;
	ufd.events = POLLIN;

	for (;;) {
		boost::this_thread::interruption_point();

		// get the next event from the keyboard
		int num;

		if ((num = poll(&ufd, 1, 250)) < 0) {
			perror("poll():");
			return;
		} else if (num > 0) {
			if (read(kfd, &c, 1) < 0) {
				perror("read():");
				return;
			}
		} else {
			if (dirty == true) {
				stopRobot();
				dirty = false;
			}

			continue;
		}

		switch (c) {
		case KEYCODE_W:
			max_tv = walk_vel_;
			speed = 1;
			turn = 0;
			dirty = true;
			break;
		case KEYCODE_S:
			max_tv = walk_vel_;
			speed = -1;
			turn = 0;
			dirty = true;
			break;
		case KEYCODE_A:
			max_rv = yaw_rate_;
			speed = 0;
			turn = 1;
			dirty = true;
			break;
		case KEYCODE_D:
			max_rv = yaw_rate_;
			speed = 0;
			turn = -1;
			dirty = true;
			break;

		case KEYCODE_W_CAP:
			max_tv = run_vel_;
			speed = 1;
			turn = 0;
			dirty = true;
			break;
		case KEYCODE_S_CAP:
			max_tv = run_vel_;
			speed = -1;
			turn = 0;
			dirty = true;
			break;
		case KEYCODE_A_CAP:
			max_rv = yaw_rate_run_;
			speed = 0;
			turn = 1;
			dirty = true;
			break;
		case KEYCODE_D_CAP:
			max_rv = yaw_rate_run_;
			speed = 0;
			turn = -1;
			dirty = true;
			break;

		default:
			max_tv = walk_vel_;
			max_rv = yaw_rate_;
			speed = 0;
			turn = 0;
			dirty = false;
		}

		//Calculate the final velocities-linear and angular, and publish on the topic
		cmdvel_.linear.x = speed * max_tv;
		cmdvel_.angular.z = turn * max_rv;
		pub_.publish(cmdvel_);
	}
}

