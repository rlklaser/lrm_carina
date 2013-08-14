/*
 *  Copyright (C) 2012, Laboratorio de Robotica Movel - ICMC/USP
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
 * @file keyboard.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Apr 26, 2013
 *
 */

#include <termios.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
	int kfd = 0;
	struct termios cooked, raw;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~(ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	char c;
	bool stop = false;
	while (!stop) {
		if (read(kfd, &c, 1) < 0) {
			perror("read():");
			exit(-1);
		}
		switch (c) {
		case 'h':
			std::cout << "um agahh" << std::endl;
			break;
		case 'q':
			stop = true;
			break;
		default:
			std::cout << "um " << c << std::endl;
			break;
		}
	}

	tcsetattr(kfd, TCSANOW, &cooked);
	return 0;
}
