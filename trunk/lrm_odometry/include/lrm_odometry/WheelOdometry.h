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
 * @file WheelOdometry.h
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 15, 2012
 *
 */

#ifndef WHEELODOMETRY_H_
#define WHEELODOMETRY_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>

//#include <atuacao/Encoders.h>

#include <lrm_description/Constants.h>
#include <lrm_msgs/Encoders.h>
#include <lrm_msgs/Velocity.h>

#include "lrm_odometry/OdometryConfig.h"


#define DEFAULT_RATE 				10
#define INF	99999999
#define DEFAULT_ROT_COV	0.00001
#define DEFAULT_POS_COV	0.00001

struct st_param
{
	int rate;

	double robot_length;
	double robot_width;
	double const_odom;
	double const_steer;
	//double encoder_to_angle_const;
	double max_inner_wheel_angle;
	double max_outer_wheel_angle;
	//double steer_encoder_at_max;
	//double wheel_perimeter;
	double wheel_diameter;
	double wheel_ray;
	double steer_bar_zero;

	bool use_imu;
	bool publish_tf;	//publish TF
	bool publish_js;	//publish Joint States (not simulated)
	bool print_info;
	bool animation_only; // only publish joint states related to animation visualization

	std::string base_odometry;
	std::string fixed_odometry;
	std::string base_footprint;
	std::string base_link;

	//bool tf_ok;
	double rot_cov;
	double pos_cov;
};

struct st_odometry
{
	double dist_f_l; //delta distance of left front wheel
	double dist_f_r; //delta distance of right front wheel

	double dist_r_l; //delta distance of left rear wheel
	double dist_r_r; //delta distance of right rear wheel

	double dist; //measured delta distance
	double dist_f; //middle front distance
	double dist_r; //middle rear distnace

	double dist_r_2; //middle rear distnace

	double R_f_l; //left front wheel ray
	double R_f_r; //right front wheel ray
	double R_f; //middle robot front ray
	double R_r_l; //left rear wheel ray
	double R_r_r; //right rear wheel ray
	double R; //middle robot rear ray (base of odometry)
	double ratio; //ratio between inner and outer rays

	double phi_i; //inner wheel steer angle
	double phi_o; //outer wheel steer angle
	double phi_l; //left whell ackerman angle
	double phi_r; //right wheel ackerman angle
	double phi; //steer angle
	double alpha; //for polar coordinate
	double new_phi;

	double v; //velocity

	//use fixed to be constant
	//double dt = (1/30.0);
	double dt;;

	//position and orientation
	double delta_x;
	double delta_y;
	double delta_theta;
	double delta_theta_2;

	//just for formulas clarity
	double L;
	double b;
	double a;
	double L2;
	double b_2;
	double k;

	double yaw;
	double initial_orientation;
	double last_orientation;

	double x,y;					/**< Holds the current X,Y position */
	double theta;				/**< Holds the current orientation */

	long int encWheelValue;
	long int encSteerValue;

	double distance;

	long int msg_counter;

	ros::Time current_time;
	ros::Time last_time;

	struct st_param p;
};


class WheelOdometry {

private:
	ros::NodeHandle nh, nh_priv;
	ros::Subscriber encoders_sub;
	ros::Subscriber imu_sub;
	ros::Publisher pose_pub;
	ros::Publisher posestamped_pub;
	ros::Publisher odom_pub;
	ros::Publisher joint_pub;
	ros::Publisher velocity_pub;
	ros::Timer odom_publisher_timer;

	tf::TransformBroadcaster odom_broadcaster;
	tf::TransformListener listener;

	sensor_msgs::JointState joint_state;
	geometry_msgs::Pose2D current_pose;
	geometry_msgs::PoseStamped current_posestamped;
	//geometry_msgs::TransformStamped base_to_odom_msg;
	tf::StampedTransform base_to_odom_msg;
	nav_msgs::Odometry odom;

	long long dbg_acc;
	boost::mutex mutex;

	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
	//void encodersCallback(const atuacao::Encoders::ConstPtr& encoder);
	void encodersCallback(const lrm_msgs::Encoders::ConstPtr& encoder);
	void calcParams();
	void getParams();
	void publishTF();
	void publishJointStates();
	bool calcTransform();
	void calcJointStates();

protected:
	struct st_odometry odo;

	virtual void calcOdometry() = 0;
	virtual void odomCallback(const ros::TimerEvent& t);
	virtual void publishPose();
	virtual void publishOdometry();
	virtual void publishVelocity();

public:
	WheelOdometry(ros::NodeHandle n);
	virtual ~WheelOdometry();

	void reconfigure(lrm_odometry::OdometryConfig &config, uint32_t level);
	void resetPose();
	void setPose(double x, double y, double theta);
	void start();
};

#endif /* WHEELODOMETRY_H_ */
