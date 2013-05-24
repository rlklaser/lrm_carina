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
 * @file two_gps_log.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jan 23, 2013
 *
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>

using namespace message_filters;

//typedef sync_policies::ExactTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> SyncPolicy;
typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> SyncPolicy;

ros::Time old_time = ros::Time(0);
geometry_msgs::Point old_pos;
double dist = 0;
double sec = 0;
bool first = true;

template<class T>
std::string time_to_str(T ros_t) {
	char buf[1024] = "";
	time_t t = ros_t.sec;
	struct tm *tms = localtime(&t);
	//strftime(buf, 1024, "%Y-%m-%d-%H-%M-%S", tms);
	strftime(buf, 1024, "%H:%M:%S", tms);
	return std::string(buf);
}

void callback(const geometry_msgs::PoseStamped::ConstPtr& pose_a, const geometry_msgs::PoseStamped::ConstPtr& pose_b) {

	/*
	 std::cout <<
	 pose_a->header.stamp << "," << pose_b->header.stamp << "," << fabs(pose_a->header.stamp.toSec() -pose_b->header.stamp.toSec()) << "," <<
	 pose_a->pose.position.x << "," << pose_b->pose.position.x << "," << fabs(pose_a->pose.position.x-pose_b->pose.position.x) << "," <<
	 pose_a->pose.position.y << "," << pose_b->pose.position.y << "," << fabs(pose_a->pose.position.y-pose_b->pose.position.y) << std::endl;
	 */

	if (!first) {
		ros::Duration dt = pose_a->header.stamp - old_time;
		dist += sqrt(pow(old_pos.x - pose_a->pose.position.x, 2) + pow(old_pos.y - pose_a->pose.position.y, 2));
		sec += dt.toSec();
		if (sec >= 1) {
			//std::cout << dist * 3.6 << "km/h" << std::endl;

			//std::cout << pose_a->header.stamp.sec << "," << dist << std::endl;

			//std::string str = pose_a->header.stamp.toString("%H-%M-%S");
			std::string str = time_to_str(pose_a->header.stamp);

			std::cout << str << "," << (dist/sec) * 3.6 << std::endl;

			dist = 0;
			sec = 0;
		}
	}
	old_time = pose_a->header.stamp;
	old_pos = pose_a->pose.position;
	first = false;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "two_gps_log");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	Subscriber<geometry_msgs::PoseStamped> sub_a(nh, "/gps_a/xsens_a/pose", 1);
	Subscriber<geometry_msgs::PoseStamped> sub_b(nh, "/gps_b/xsens_b/pose", 1);

	Synchronizer<SyncPolicy> sync(SyncPolicy(1), sub_a, sub_b);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	ros::spin();

	return 0;
}
