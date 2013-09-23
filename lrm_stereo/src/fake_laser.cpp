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
 * @file fake_laser.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Sep 23, 2013
 *
 */

int main(int argc, char** argv) {
	ros::init(argc, argv, "fake_cloud");
	ros::NodeHandle n;
	Hemisfere hemi(n);

	if (!hemi.isRandonGen()) {
		hemi.generateFixedCloud();
	}

	ros::Rate loop_rate(hemi.getRate());
	while (n.ok()) {
		if (hemi.isRandonGen()) {
			hemi.generateRandonCloud();
		} else {
			hemi.publish();
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
}
