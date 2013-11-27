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
 * @file viz_gazebo_models_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 26, 2013
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

struct st_model {
	std::string name;
	geometry_msgs::Pose pose;
	geometry_msgs::Twist twist;
};

ros::Publisher markersPub;
ros::Publisher markerPub;
ros::Subscriber modelSub;

bool published;

//void modelsCallback(gazebo_msgs::ModelStates::ConstPtr msg) {
void modelsCallback(gazebo_msgs::LinkStates::ConstPtr msg) {

	if(published) return;
	published = true;

	std::vector<st_model> models;

	for (unsigned i = 0; i < msg->name.size(); i++) {
		//std::cout << msg->name[i] << std::endl;
		if (strncmp(msg->name[i].c_str(), "finihed", 7) == 0) {
			struct st_model model;
			model.name = msg->name[i];
			model.pose = msg->pose[i];
			model.twist = msg->twist[i];

			if(model.pose.position.x == 0 && model.pose.position.y == 0 && model.pose.position.z == 0) {
				continue;
			}

			models.push_back(model);
		}

		if (strncmp(msg->name[i].c_str(), "carina::base_footprint", 22) == 0) {
			std::cout << "x: " << msg->pose[i].position.x << " y: " <<  msg->pose[i].position.y << std::endl;
		}
	}

	visualization_msgs::MarkerArray markers;
	markers.markers.resize(models.size());
	ros::Time rostime = ros::Time::now();

	for (unsigned i = 0; i < models.size(); ++i) {
		markers.markers[i].header.frame_id = "/map";
		markers.markers[i].header.stamp = rostime;
		markers.markers[i].ns = "trees";//models[i].name;
		markers.markers[i].id = i;
		//markers.markers[i].type = visualization_msgs::Marker::CYLINDER;
		markers.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
		markers.markers[i].action = visualization_msgs::Marker::ADD;

		//markers.markers[i].scale.x = 0.4;
		//markers.markers[i].scale.y = 0.4;
		//markers.markers[i].scale.z = 3;

		markers.markers[i].scale.x = 1;
		markers.markers[i].scale.y = 1;
		markers.markers[i].scale.z = 1;
		markers.markers[i].pose = models[i].pose;

		//markers.markers[i].color.r = 0.3;
		//markers.markers[i].color.g = 1;
		//markers.markers[i].color.b = 0.3;
		//markers.markers[i].color.a = 1;

		markers.markers[i].mesh_resource = "package://lrm_description/models/trees/finihed/meshes/finihed.dae";
		markers.markers[i].text = models[i].name;
		markers.markers[i].mesh_use_embedded_materials = true;
		//markers.markers[i].pose.orientation = models[i].twist;

		//markerPub.publish(markers.markers[i]);
	}

	std::cout << "markers : " << models.size() << std::endl;

	markersPub.publish(markers);
	modelSub.shutdown();

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "viz_gazebo_models_node");
	ros::NodeHandle nh;

	published = false;
	markersPub = nh.advertise<visualization_msgs::MarkerArray>("viz_gazebo_models_array", 1, true);
	//markerPub = nh.advertise<visualization_msgs::Marker("viz_gazebo_models", 1, true);
	//modelSub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &modelsCallback);
	modelSub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, &modelsCallback);

	ros::spin();
}
