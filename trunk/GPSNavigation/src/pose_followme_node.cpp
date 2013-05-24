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
 * @file pose_followme_node.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 3, 2013
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <queue.h>


typedef struct {
	double x;
	double y;
	double v;
	double theta;
	ros::Time ttl;
	long id;
} PointV;

ros::Subscriber reached_sub;
ros::Publisher goal_pub;
ros::Timer timeout_timer;

PointV _current_point;
std::queue<PointV> _gps_points;
long _current_id;
double _current_velocity;
geometry_msgs::PoseStampedPtr _current_pose;

PointV new_point(double x, double y, double theta, ros::Time ttl) {
	PointV pt;

	pt.ttl = ttl;
	pt.x = x;
	pt.y = y;
	pt.theta = theta;
	pt.v = _current_velocity;
	pt.id = _current_id++;

	_gps_points.push(pt);

	ROS_WARN("new way point %f", ttl.toSec());

	return pt;
}

void callback_pose(const geometry_msgs::PoseStampedPtr pose) {
	ros::Time now = ros::Time::now();
	//ros::Time now = pose->header.stamp;

	if (_first_point) {
		_first_point = false;
		_relative_x = pose->pose.position.x;
		_relative_y = pose->pose.position.y;
	}

	if ((now - _last) > ros::Duration(1 / p_waypointHZ)) {
		_last = now;
		new_point(pose->pose.position.x, pose->pose.position.y, now + ros::Duration(p_waypointTTL));
	}
	_current_pose = pose;
}

void publishFromSignal(PointV& point)
{
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "/goal";
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta);
    goal.pose.position.x = pose.x;
    goal.pose.position.y = pose.y;

    goal_pub.publish(goal);

    //TODO: period based on max velocity and distance
    //timeout_timer.setPeriod()
    timeout_timer.start();
}

void reachedReceived(const GPSNavigation::Goal::ConstPtr& msg)
{
	timeout_timer.stop();

	pose_x=odom.pose.pose.position.x;
	pose_y=odom.pose.pose.position.y;

	//search memory for a better signal, else goto next kernel position
	//std::cout << "REACHED (" << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ")" << std::endl;

	pose_x=msg->pose.x;
	pose_y=msg->pose.y;

	double area = fabs((pose_x-best_signal.pose.x)*(pose_y-best_signal.pose.y));

	std::cout << "REACHED (" << pose_x << ", " << pose_y << ") a:" << area << std::endl;


	publishFromSignal();
}

void timeoutCallback(const ros::TimerEvent& t)
{
	timeout_timer.stop();
	ROS_WARN("timeout... new position");
	//publishFromKernel();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pose_followme_node");
	ros::NodeHandle nh;

	double timeout = 1;

    goal_pub = n.advertise<geometry_msgs::PoseStamped>(n.getNamespace() + "/move_base_simple/goal", 5, true);
    timeout_timer = n.createTimer(ros::Duration(timeout), timeoutCallback);
    reached_sub = n.subscribe<lrm_rbb_grupo_c::Goal>("move_base_simple/reached", 2, reachedReceived);


    ros::spin();
}
