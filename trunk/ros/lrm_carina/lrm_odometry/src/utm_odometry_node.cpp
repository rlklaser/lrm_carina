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
#include <tf/tf.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>

using namespace gps_common;

static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;

double latitude, longitude;
double pos_cov;
double rot_cov;

bool first = true;
double northing_start, easting_start;
nav_msgs::Odometry odom;
//double theta;
double yaw;
double initial_orientation;
//double last_orientation;
//geometry_msgs::Quaternion imu_orientation;

tf::TransformBroadcaster* tf_broadcaster;

double heading;
double last_x, last_y;

bool imu_ok;
bool _show_info;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	yaw = tf::getYaw(msg->orientation);
	//imu_orientation = msg->orientation;
	/*
	 odo.imu_ang_vel_x = msg->angular_velocity.x;
	 odo.imu_ang_vel_y = msg->angular_velocity.y;
	 odo.imu_ang_vel_z = msg->angular_velocity.z;
	 */
	//ROS_INFO_STREAM("angle:" << angles::to_degrees(yaw));
	imu_ok = true;
}

void fixCallback(const sensor_msgs::NavSatFixConstPtr& fix) {
	if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
		ROS_INFO_THROTTLE(0.2, "No fix.");
		return;
	}

	if (fix->header.stamp == ros::Time(0)) {
		return;
	}

	if (!imu_ok) {
		return;
	}

	double northing, easting;

	std::string zone;

	LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);
	//UTM(fix->latitude, fix->longitude, &northing, &easting);

	if (first) {
		first = false;
		latitude = fix->latitude;
		longitude = fix->longitude;
		//LLtoUTM(latitude, longitude, northing_start, easting_start, zone);
		//UTM(latitude, longitude, &northing_start, &easting_start);
		//theta = 0;
		northing_start = northing;
		easting_start = easting;

		initial_orientation = yaw;
		//tf::Quaternion qtu(0.0187677443027, 0.0216172095388, 0.600740909576,  0.798931062222);
		//ROS_INFO_STREAM("angle:" << qtu.getAngle());
		//heading = 0;
		heading = initial_orientation;

		ROS_INFO_STREAM("angle:" << angles::to_degrees(yaw));
		ROS_INFO_STREAM("utm odometry: initial lat:" << latitude << " lon:" << longitude);

		return;
	}

	if (odom_pub) {
		odom.header.stamp = ros::Time::now(); //fix->header.stamp;

		if (frame_id.empty())
			odom.header.frame_id = fix->header.frame_id;
		else
			odom.header.frame_id = "/" + frame_id;

		odom.child_frame_id = child_frame_id;

		double pos_x = (easting - easting_start);
		double pos_y = (northing - northing_start);

		//East North Up
		//double pos_y = -(easting - easting_start);
		//double pos_x = (northing - northing_start);

		//heading = atan2(pos_y-odom.pose.pose.position.y, pos_x-odom.pose.pose.position.x);
		//heading = atan2(pos_y, pos_x);

//		double delta_y = pos_y - last_y;
//		double delta_x = pos_x - last_x;

		double delta_y = pos_y - last_y;
		double delta_x = pos_x - last_x;

		//if(delta_y+delta_x!=0)

		double dist = sqrt(delta_x*delta_x + delta_y*delta_y);

		if(dist>0.5)
		{
		//if (delta_y + delta_x > 1) {
			double new_heading = atan2(delta_y, delta_x);
			//double new_heading = atan2(pos_y, pos_x);
			heading = new_heading;
			//heading = new_heading + (new_heading - heading);
			last_x = pos_x;
			last_y = pos_y;
			//abs(p1.y-p2.y)/sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
		}
		//heading = angles::normalize_angle_positive(heading);

		//ROS_INFO_STREAM("heading:" << angles::to_degrees(heading));

		//theta  = angles::normalize_angle(angles::from_degrees(90) - heading);

		odom.pose.pose.position.x = pos_x;
		odom.pose.pose.position.y = pos_y;
		//odom.pose.pose.position.z = theta;//fix->altitude;

		/*
		 tf::Quaternion qt = tf::createQuaternionFromYaw(theta);
		 */

		//yaw = 0;
		//tf::Quaternion qt = tf::createQuaternionFromYaw(yaw /*- initial_orientation*/);
		//tf::Quaternion qt = tf::createQuaternionFromYaw(yaw);

		double imu_heading = yaw + angles::from_degrees(90);
		double pos_heading = heading;// angles::from_degrees(90) - heading;

		imu_heading = angles::normalize_angle_positive(imu_heading);
		pos_heading = angles::normalize_angle_positive(pos_heading);

		if(_show_info) {
			ROS_INFO_STREAM("headings:(imu)" << angles::to_degrees(imu_heading) << ":(pos)" << angles::to_degrees(pos_heading));
		}

		//tf::Quaternion qt = tf::createQuaternionFromYaw(imu_heading);
		tf::Quaternion qt = tf::createQuaternionFromYaw(pos_heading);
		//tf::Quaternion qt = tf::createQuaternionFromYaw(heading);

		odom.pose.pose.orientation.x = qt.x();
		odom.pose.pose.orientation.y = qt.y();
		odom.pose.pose.orientation.z = qt.z();
		odom.pose.pose.orientation.w = qt.w();

		/*
		 odom.pose.pose.orientation.x = 1;
		 odom.pose.pose.orientation.y = 0;
		 odom.pose.pose.orientation.z = 0;
		 odom.pose.pose.orientation.w = 0;
		 */

		//odom.pose.pose.orientation = imu_orientation;
		/*
		 // Use ENU covariance to build XYZRPY covariance
		 boost::array<double, 36> covariance = {{
		 pos_cov, pos_cov, pos_cov, 0, 0, 0,
		 pos_cov, pos_cov, pos_cov, 0, 0, 0,
		 pos_cov, pos_cov, pos_cov, 0, 0, 0,
		 0, 0, 0, rot_cov, 0, 0,
		 0, 0, 0, 0, rot_cov, 0,
		 0, 0, 0, 0, 0, rot_cov }};
		 */

		boost::array<double, 36> covariance = { { pos_cov, 0, 0, 0, 0, 0, // covariance on gps_x
				0, pos_cov, 0, 0, 0, 0, // covariance on gps_y
				0, 0, pos_cov, 0, 0, 0, // covariance on gps_z
				0, 0, 0, rot_cov, 0, 0, // covariance on rot x
				0, 0, 0, 0, rot_cov, 0, // covariance on rot y
				0, 0, 0, 0, 0, rot_cov } // covariance on rot z
		};
		odom.pose.covariance = covariance;

		odom_pub.publish(odom);

		//tf::Quaternion qtt = tf::createQuaternionFromRPY(yaw, 0, 0);
		//tf::Quaternion qtt = tf::createQuaternionFromYaw(-yaw);

		//tf::Transform trans_odom_encoder(qtt, tf::Vector3(0.0, 0.0, 0.0));
		//tf::StampedTransform trans_odom_base_st(trans_odom_encoder, ros::Time::now(), "earth", "map");

		//tf_broadcaster->sendTransform(trans_odom_base_st);
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "utm_odometry_node");
	ros::NodeHandle node;
	ros::NodeHandle priv_node("~");

	tf_broadcaster = new tf::TransformBroadcaster();

	priv_node.param<std::string>("frame_id", frame_id, "odom_combined");
	priv_node.param<std::string>("child_frame_id", child_frame_id, "base_footprint");
	priv_node.param<double>("rot_cov", rot_cov, 99999);
	priv_node.param<double>("pos_cov", pos_cov, 0.001);
	priv_node.param<double>("latitude", latitude, 0.0);
	priv_node.param<double>("longitude", longitude, 0.0);
	priv_node.param<bool>("show_info", _show_info, false);

	odom_pub = node.advertise<nav_msgs::Odometry>(priv_node.getNamespace() + "/odom", 10);

//yaw = 0;
	imu_ok = false;

	ros::Subscriber fix_sub = node.subscribe("fix", 1, fixCallback);
	ros::Subscriber imu_sub = node.subscribe("imu", 1, imuCallback);

	ROS_INFO_STREAM("utm odometry: started in lat:" << latitude << " lon:" << longitude);

	ros::spin();

	delete tf_broadcaster;
}
