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
 * @file transform_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Oct 8, 2012
 *
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
	static tf::TransformBroadcaster br;
//	tf::Transform transform;
//	transform.setOrigin(tf::Vector3(msg->pose.position.x-199800, msg->pose.position.y-7564220, msg->pose.position.z-840));
//	transform.setRotation(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
//	br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "world"));

	tf::Transform tt;
	tt.setOrigin(tf::Vector3(0, 0, .3));
	tt.setRotation(tf::Quaternion(0, -7*M_PI/180, 0, 1));
	br.sendTransform(tf::StampedTransform(tt, msg->header.stamp, "world", "base_link"));

//	tf::Transform t2;
//	t2.setOrigin(tf::Vector3(.2, 0, 0));
//	t2.setRotation(tf::Quaternion(0, 0, 0, 1));
//	br.sendTransform(tf::StampedTransform(t2, msg->header.stamp, "base_foot", "base_link"));

//	tf::Transform t3;
//	t3.setOrigin(tf::Vector3(0, 0, .1));
//	t3.setRotation(tf::Quaternion(0, 0, 0, 1));
//	br.sendTransform(tf::StampedTransform(t3, msg->header.stamp, "base_link", "base_imu"));

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "transform_node");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("lse_xsens_mti/xsens/pose", 10, &poseCallback);

	//ros::Subscriber ss = node.subscribe("lse_xsens_mti/xsens/")
	ros::spin();
	return 0;
}

/*
int main(int argc, char** argv)
{
	ros::init(argc, argv, "transform_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	std::string node_namespace = nh_priv.getNamespace();
	tf::TransformBroadcaster cam_broadcaster;
	tf::TransformListener base_listener;

    tf::StampedTransform tf_base_to_cam;
    try{
        base_listener.lookupTransform("base_link", "world", ros::Time(0), tf_base_to_cam);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    btTransform T_odom_imu(btQuaternion(quaternionX,quaternionY,quaternionZ,quaternionW),btVector3(current_position.x,current_position.y, current_position.z));
	tf::StampedTransform T_odom_base_st(T_odom_imu, now, mRosNamespace + ODOMETRY_FRAME_ID, mRosNamespace + BASE_LINK_FRAME_ID);
	T_odom_base_st *= T_base_imu.inverse();
	geometry_msgs::TransformStamped base_to_odom_msg;
	tf::transformStampedTFToMsg(T_odom_base_st, base_to_odom_msg);
	if(qt.quaternion.x != 0.0 && qt.quaternion.y != 0.0 && qt.quaternion.z != 0.0 && qt.quaternion.w != 0.0)
		odom_broadcaster.sendTransform(base_to_odom_msg);

    ros::Rate r(30);

    while(ros::ok())
    {

    	ros::spin();
    }

}
*/
