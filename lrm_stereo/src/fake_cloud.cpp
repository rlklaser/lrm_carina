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
 * @file fake_cloud.cpp
 * @brief 
 * @author Diego Gomes
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 13, 2013
 *
 */

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>
#include <math.h>      /* time */

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

#define R 5.0
#define DENSITY_DEFAULT 5

class Hemisfere {
private:
	ros::NodeHandle nh, nh_priv;
	int density, rate;
	bool isRandon;
	double rayd;
	float ray;

	PointCloud::Ptr msg;
	ros::Publisher pub;
public:
	Hemisfere(ros::NodeHandle n);
	virtual ~Hemisfere();
	void generateFixedCloud();
	void generateRandonCloud();
	void publish();
	bool isRandonGen();
	int getRate();
};

Hemisfere::Hemisfere(ros::NodeHandle n) :
		nh(n), nh_priv("~"), msg(new PointCloud) {

	std::string frame_id;

	nh_priv.param("rate", rate, 10);
	nh_priv.param("isRandon", isRandon, true);
	nh_priv.param("density", density, DENSITY_DEFAULT);
	nh_priv.param("ray", rayd, R);
	nh_priv.param<std::string>("frame_id", frame_id, "base_link");

	ray = (float) rayd;

	msg->header.frame_id = frame_id;

	pub = nh.advertise<PointCloud>("cloud", 1);
	srand((unsigned) time(0));
}

Hemisfere::~Hemisfere() {
} //{ delete msg;}

void Hemisfere::generateFixedCloud() {
	msg->header.stamp = ros::Time::now();
	msg->height = msg->width = (int) sqrt(density / 2 * M_PI * R * R);

	for (int i = 0; i < (int)msg->height; i++) {
		for (int j = 0; j < (int)msg->width; j++) {

			float x = (((float) i / (float) msg->height) - 0.5) * 2 * ray;
			float y = (((float) j / (float) msg->width) - 0.5) * 2 * ray;

			msg->points.push_back(pcl::PointXYZ(x, y, sqrt(-(x * x) - (y * y) + (ray * ray))));
		}
	}
	pub.publish(msg);
}

void Hemisfere::generateRandonCloud() {
	msg->header.stamp = ros::Time::now();
	msg->height = msg->width = (int) sqrt(density / 2 * M_PI * R * R);

	for (int i = 0; i < (int)msg->height; i++) {
		for (int j = 0; j < (int)msg->width; j++) {

			float x = -ray + (float) rand() / ((float) RAND_MAX / (ray - (-ray)));
			float y = -ray + (float) rand() / ((float) RAND_MAX / (ray - (-ray)));

			msg->points.push_back(pcl::PointXYZ(x, y, sqrt(-(x * x) - (y * y) + (ray * ray))));
		}
	}
	pub.publish(msg);
	msg->points.clear();
}

void Hemisfere::publish() {
	msg->header.stamp = ros::Time::now();
	pub.publish(msg);
}

bool Hemisfere::isRandonGen() {
	return isRandon;
}

int Hemisfere::getRate() {
	return rate;
}

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
