/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

/**
 *
 * rlklaser 2013-01-29
 *
 * Modified:
 *  - initial lat lon to start odom in (0,0)
 *  - covariances passed by param
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>

using namespace gps_common;

static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;

double latitude, longitude;
double pos_cov;
double rot_cov;

bool first = true;
double northing_start, easting_start;

void callback(const sensor_msgs::NavSatFixConstPtr& fix) {
	if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
		ROS_INFO("No fix.");
		return;
	}

	if (fix->header.stamp == ros::Time(0)) {
		return;
	}

	double northing, easting;

	std::string zone;

	LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

	if(first) {
		first = false;
		latitude = fix->latitude;
		longitude = fix->longitude;
		LLtoUTM(latitude, longitude, northing_start, easting_start, zone);
	}

	if (odom_pub) {
		nav_msgs::Odometry odom;
		odom.header.stamp =  ros::Time::now();//fix->header.stamp;

		if (frame_id.empty())
			odom.header.frame_id = fix->header.frame_id;
		else
			odom.header.frame_id = frame_id;

		odom.child_frame_id = child_frame_id;

		odom.pose.pose.position.x = easting - easting_start;
		odom.pose.pose.position.y = northing - northing_start;
		odom.pose.pose.position.z = 0;//fix->altitude;

		odom.pose.pose.orientation.x = 0;
		odom.pose.pose.orientation.y = 0;
		odom.pose.pose.orientation.z = 0;
		odom.pose.pose.orientation.w = 1;

		/*
		// Use ENU covariance to build XYZRPY covariance
		boost::array<double, 36> covariance = {{
				pos_cov, pos_cov, pos_cov,
				0, 0, 0, pos_cov, pos_cov, pos_cov,
				0, 0, 0, pos_cov, pos_cov, pos_cov,
				0, 0, 0, 0, 0, 0, rot_cov, 0,
				0, 0, 0, 0, 0, rot_cov, 0, 0, 0, 0, 0, 0, rot_cov }};

		*/

		boost::array<double, 36> covariance = {
				{pos_cov, 0, 0, 0, 0, 0,  // covariance on gps_x
				0, pos_cov, 0, 0, 0, 0,  // covariance on gps_y
				0, 0, 99999.0, 0, 0, 0,  // covariance on gps_z
				0, 0, 0, rot_cov, 0, 0,  // large covariance on rot x
				0, 0, 0, 0, rot_cov, 0,  // large covariance on rot y
				0, 0, 0, 0, 0, rot_cov}  // large covariance on rot z
		};
		odom.pose.covariance = covariance;

		odom_pub.publish(odom);
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "utm_odometry_node");
	ros::NodeHandle node;
	ros::NodeHandle priv_node("~");

	priv_node.param<std::string>("frame_id", frame_id, "");
	priv_node.param<std::string>("child_frame_id", child_frame_id, "");
	priv_node.param<double>("rot_cov", rot_cov, 99999.0);
	priv_node.param<double>("pos_cov", pos_cov, 0.0001);
	priv_node.param<double>("latitude", latitude, 0.0);
	priv_node.param<double>("longitude", longitude, 0.0);

	odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);

	ros::Subscriber fix_sub = node.subscribe("fix", 10, callback);

	ROS_INFO_STREAM("started in lat:" << latitude << " lon:" << longitude);

	ros::spin();
}

