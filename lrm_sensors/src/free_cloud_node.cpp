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
 * @file free_cloud_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Apr 3, 2013
 *
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#define MAX_POINTS 20000  //Maximum points to display
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud; //create a pointcloud using XYZ and I(ntensity)

float Arr[541];

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array);

int main(int argc, char** argv) {
	ros::init(argc, argv, "free_cloud_node");
	ros::NodeHandle n;

	//Publish
	ros::Publisher pub = n.advertise<PointCloud>("free_cloud", 100);

	//Subscribe
	ros::Subscriber sub3 = n.subscribe("sonarPCarray", 100, arrayCallback);

	//Set loop rate (times per second)
	ros::Rate r(30);

	//Create a pointcloud message.
	PointCloud::Ptr msg(new PointCloud);
	msg->header.frame_id = "stereo_camera";
	msg->height = 1;
	msg->width = MAX_POINTS;
	msg->points.resize(msg->width * msg->height);

	ros::spinOnce();

	int counter = 0;

	while (ros::ok()) {

		for (int i = 0; i < 90; i++) {
			//threshold the data, only display if reading above 90.0
			// also ignore the first 10, as it's noise from the sonar.
			if ((i >= 10) && (Arr[3 + (i * 4)] >= 90.0)) {

				msg->points[counter].x = Arr[0 + (i * 4)]; //X point
				msg->points[counter].y = Arr[2 + (i * 4)]; //Y point (unused)
				msg->points[counter].z = Arr[1 + (i * 4)]; //Z point

				msg->points[counter].intensity = Arr[3 + (i * 4)]; //sensor reading.

				msg->header.stamp = ros::Time::now();

				pub.publish(msg);
				counter++;
				i = 90;
			}

		}

		ros::spinOnce();
		r.sleep();

		//Limit the total number of points in the point cloud, can really kill rviz if too high.
		if (counter >= MAX_POINTS) {
			counter = 0;
		}

	}
}

/*************************************************
 ** Returns the array from sonarInjector  	**
 *************************************************/

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array) {

	for (std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
		Arr[i] = *it;
		i++;
	}
	return;
}
