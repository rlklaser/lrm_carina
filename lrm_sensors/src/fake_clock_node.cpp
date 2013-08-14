/*
 *  Copyright (C) 2012-2013, Laboratorio de Robotica Movel - ICMC/USP
 *  Rafael Luiz Klaser <rlklaser@gmail.com>
 *  http://lrm.icmc.usp.br
 *
 *  Apoio FAPESP: 2012/04555-4
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file fake_clock_node.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jul 21, 2013
 *
 */

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/package.h>

#include <rosgraph_msgs/Clock.h>

#include <sys/time.h>
#include <time.h>

//#include <chrono>

#include <termios.h>
#include <pthread.h>

bool paused;
bool quit;
int kfd = 0;
struct termios cooked;

void init_keyboard() {
	struct termios raw;
	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~(ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
}

void fini_keyboard() {
	tcsetattr(kfd, TCSANOW, &cooked);
}


void* keyboard_watch(void *arg) {
	char c;

	for(;;) {
		if (read(kfd, &c, 1) < 0) {
			break;
		}
		else {
			if (c == 'p') {
				paused = !paused;
				printf("[%s]\n", (paused ? "PAUSED" : "UNPAUSED"));
			}
			if (c == 'q') {
				quit = true;
				break;
			}
			//else {
			//	printf("%d\n", c);
			//}
		}
	}
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "fake_clock_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	double rate;
	nh_priv.param<double>("rate", rate, 1000);

	nh.setParam("/use_sim_time", true);

	struct timeval tv;
	ros::Rate hz(rate);
	rosgraph_msgs::Clock clock;

	ROS_INFO_STREAM("publishing /clock at " << rate << " Hz - [p - pause/unpause, q - quit");

	ros::Publisher pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);

	paused = false;
	quit = false;
	pthread_t thread;

	//init_keyboard();

	pthread_create(&thread, 0, keyboard_watch, 0);

	//long last = 0;
	//long tsec = 0;

	while (ros::ok() && !quit) {
		gettimeofday(&tv, 0);

		clock.clock.fromNSec((uint64_t)tv.tv_sec*1000000000ull + (uint64_t)(tv.tv_usec*1000));

		if (!paused) {
			pub.publish(clock);
		}

		//tsec = clock.clock.toSec();
		//if(tsec!=last) {
		//	printf("\r [%s] Time: %13.6f \r", (paused ? "PAUSED" : "CLOCK"), tsec);
		//	last = tsec;
		//}

		usleep(1000000.0 / rate);
	}

	pthread_exit(&thread);

	//fini_keyboard();

	return 0;
}
