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
 * @file WheelOdometry.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 15, 2012
 *
 */

#include "lrm_odometry/WheelOdometry.h"

WheelOdometry::WheelOdometry(ros::NodeHandle n) :
		nh(n), nh_priv("~") {

	getParams();
	ROS_INFO_STREAM(
			"imu:" << odo.p.use_imu <<
			" tf:" << odo.p.publish_tf <<
			" js:" << odo.p.publish_js);

	// Set the covariance matrixes relate odom message
	odom.pose.covariance.elems[0] = odo.p.pos_cov;
	odom.pose.covariance.elems[7] = odo.p.pos_cov;
	odom.pose.covariance.elems[14] = odo.p.pos_cov;
	odom.pose.covariance.elems[21] = odo.p.pos_cov;
	odom.pose.covariance.elems[28] = odo.p.pos_cov;
	odom.pose.covariance.elems[35] = odo.p.pos_cov;

	odom.twist.covariance.elems[0] = odo.p.rot_cov;
	odom.twist.covariance.elems[7] = odo.p.rot_cov;
	odom.twist.covariance.elems[14] = odo.p.rot_cov;
	odom.twist.covariance.elems[21] = odo.p.rot_cov;
	odom.twist.covariance.elems[28] = odo.p.rot_cov;
	odom.twist.covariance.elems[35] = odo.p.rot_cov;

	odo.initial_orientation = INF;
	odo.current_time = ros::Time(0);

	if (!odo.p.animation_only) {
		joint_state.name.push_back(std::string("joint_back_left_wheel"));
		joint_state.position.push_back(0);
		joint_state.name.push_back(std::string("joint_back_right_wheel"));
		joint_state.position.push_back(0);
		joint_state.name.push_back(std::string("joint_front_left_wheel"));
		joint_state.position.push_back(0);
		joint_state.name.push_back(std::string("joint_front_right_wheel"));
		joint_state.position.push_back(0);
		joint_state.name.push_back(std::string("joint_front_left_wheel_dir"));
		joint_state.position.push_back(0);
		joint_state.name.push_back(std::string("joint_front_right_wheel_dir"));
		joint_state.position.push_back(0);
	}
	joint_state.name.push_back(std::string("joint_steering_wheel"));
	joint_state.position.push_back(0);

	if (odo.p.use_imu) {
		imu_sub = nh.subscribe<sensor_msgs::Imu>("imu_data", 2, &WheelOdometry::imuCallback, this);
	}
	//encoders_sub = nh.subscribe<atuacao::Encoders> ("encoders", 2, &WheelOdometry::encodersCallback, this);
	encoders_sub = nh.subscribe<lrm_msgs::Encoders>("encoders", 2, &WheelOdometry::encodersCallback, this);

	/// publish with 1 sec buffer
	pose_pub = nh.advertise<geometry_msgs::Pose2D>(nh_priv.getNamespace() + "/pose2d", odo.p.rate);
	posestamped_pub = nh.advertise<geometry_msgs::PoseStamped>(nh_priv.getNamespace() + "/pose", odo.p.rate);
	odom_pub = nh.advertise<nav_msgs::Odometry>(nh_priv.getNamespace() + "/odom", odo.p.rate);
	velocity_pub = nh.advertise<lrm_msgs::Velocity>(nh_priv.getNamespace() + "/velocity", 1);

	joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

	//if(odo.p.publish_tf) {
	odom_publisher_timer = n.createTimer(ros::Duration(1.0 / odo.p.rate), &WheelOdometry::odomCallback, this, false, false);
	//odom_publisher_timer.start();
	//}
}

/**
 * Odometry destructor.
 */
WheelOdometry::~WheelOdometry() {
}


void WheelOdometry::start() {
	odom_publisher_timer.start();
}

void WheelOdometry::getParams() {
	nh_priv.param("max_angle_left", odo.p.max_inner_wheel_angle, 32.5);
	nh_priv.param("max_angle_right", odo.p.max_outer_wheel_angle, 32.5);
	nh_priv.param("wheel_diameter", odo.p.wheel_diameter, DEFAULT_WHEEL_DIAMETER);
	nh_priv.param("robot_length", odo.p.robot_length, DEFAULT_ROBOT_LENGHT);
	nh_priv.param("robot_width", odo.p.robot_width, DEFAULT_ROBOT_WIDTH);
	nh_priv.param("steer_bar_zero", odo.p.steer_bar_zero, 0.0);

	nh_priv.param("print_info", odo.p.print_info, false);
	nh_priv.param("use_imu", odo.p.use_imu, true);
	nh_priv.param("publish_tf", odo.p.publish_tf, true);
	nh_priv.param("publish_js", odo.p.publish_js, false);
	nh_priv.param("animation_only", odo.p.animation_only, false);
	nh_priv.param("rate", odo.p.rate, DEFAULT_RATE);

	nh_priv.param("rot_cov", odo.p.rot_cov, DEFAULT_ROT_COV);
	nh_priv.param("pos_cov", odo.p.pos_cov, DEFAULT_POS_COV);

	nh_priv.param("base_odometry", odo.p.base_odometry, std::string("base_odometry"));
	nh_priv.param("fixed_odometry", odo.p.fixed_odometry, std::string("odom"));
	nh_priv.param("base_footprint", odo.p.base_footprint, std::string("base_footprint"));
	nh_priv.param("base_link", odo.p.base_link, std::string("base_link"));

	std::string tf_prefix;
	nh.param<std::string>("tf_prefix", tf_prefix, "");

	//ROS_INFO_STREAM(tf_prefix << std::endl);

	joint_state.header.frame_id = tf_prefix + "/" + odo.p.base_link;
	odom.header.frame_id = tf_prefix + "/"+ odo.p.fixed_odometry;
	odom.child_frame_id = tf_prefix + "/" + odo.p.base_footprint;

	current_posestamped.header.frame_id = tf_prefix + "/" + odo.p.base_footprint;

	calcParams();
}

void WheelOdometry::calcParams() {
	odo.p.const_odom = M_PI * odo.p.wheel_diameter / WHEEL_ENCODER_RESOLUTION;
	odo.p.const_steer = (odo.p.max_inner_wheel_angle + odo.p.max_outer_wheel_angle) / 2 * M_PI / 180 / STEER_ENCODER_RESOLUTION;
	//just for formulas clarity
	odo.L = odo.p.robot_length;
	odo.b = odo.p.robot_width;
	odo.L2 = odo.L * odo.L;
	odo.b_2 = odo.b / 2;
	odo.p.wheel_ray = odo.p.wheel_diameter / 2;
}

void WheelOdometry::reconfigure(lrm_odometry::OdometryConfig &config, uint32_t level) {
	odo.p.max_inner_wheel_angle = config.max_angle_left;
	odo.p.max_outer_wheel_angle = config.max_angle_right;
	odo.p.robot_length = config.robot_lenght;
	odo.p.robot_width = config.robot_width;
	odo.p.wheel_diameter = config.wheel_diameter;

	calcParams();

	ROS_WARN("node reconfigured");
}

void WheelOdometry::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	if (odo.initial_orientation == INF) {
		odo.initial_orientation = tf::getYaw(msg->orientation);
		odo.last_orientation = odo.initial_orientation;
	}
	odo.yaw = tf::getYaw(msg->orientation) - odo.initial_orientation;
}

/**
 * This method gets the encoders values and compute robot odometry.
 * @param encoder Pointer to encoder message.
 */
//void WheelOdometry::encodersCallback(const atuacao::Encoders::ConstPtr& encoder)
void WheelOdometry::encodersCallback(const lrm_msgs::Encoders::ConstPtr& encoder) {
	/*
	 ros::Time stamp;

	 stamp = encoder->header.stamp;
	 //stamp = ros::Time::now();

	 if(odo.current_time == ros::Time(0)) {
	 odo.last_time = stamp;
	 }
	 odo.current_time = stamp;
	 */
	odo.encWheelValue += encoder->wheel.relative;
	odo.encSteerValue = encoder->steering.absolute;

	dbg_acc+=encoder->wheel.relative;
}

void WheelOdometry::odomCallback(const ros::TimerEvent& t) {
	ros::Time stamp;

	//stamp = encoder->header.stamp;
	stamp = ros::Time::now();

	if (odo.current_time == ros::Time(0)) {
		odo.last_time = stamp;
	}
	odo.current_time = stamp;

	odo.dt = (odo.current_time - odo.last_time).toSec();

	//odo.encWheelValue = encoder->relative.front();
	//odo.encSteerValue = encoder->absolute.front();
	//odo.encWheelValue = encoder->wheel.relative;
	//odo.encSteerValue = encoder->steering.absolute;

	boost::unique_lock<boost::mutex> scoped_lock(mutex);
	calcOdometry();

	//velocity
	if (odo.dt != 0) {
		odo.v = odo.dist / odo.dt;
	} else {
		odo.v = 0;
	}
	odo.distance += odo.dist;
	if (odo.p.use_imu) {
		odo.theta = odo.yaw;
	} else {
		odo.theta += odo.delta_theta;
	}
	odo.theta = fmod(odo.theta, (2.0 * M_PI));
	//(x,y) from polar coordinate (alpha, rho)
	odo.alpha = odo.theta + odo.delta_theta / 2;
	odo.delta_x = odo.dist * cos(odo.alpha);
	odo.delta_y = odo.dist * sin(odo.alpha);
	odo.x += odo.delta_x;
	odo.y += odo.delta_y;
	odo.last_time = odo.current_time;
	odo.last_orientation = odo.yaw;

	if (odo.p.print_info) {
		ROS_INFO_STREAM(
			"v:" << odo.v*3.6 <<
			" dt:" << odo.distance <<
			" phi:" << odo.phi*180/M_PI <<
			//" nphi:" << odo.new_phi*180/M_PI <<
			//" d:" << dist_r <<
			//" d2: " << dist_r_2 <<
			" (phi" <<
			" L:" << odo.phi_l*180/M_PI <<
			" R:" << odo.phi_r*180/M_PI <<
			")(d" <<
			" L:" << odo.dist_f_l <<
			" R:" << odo.dist_f_r <<
			")" <<

			" enc: " << odo.encSteerValue <<
			" / " << odo.encWheelValue <<
			//<< " rl:" << R_f_l
			//<< " rr:" << R_f_r
			//<< " r:" << ratio

			" dth:" << odo.delta_theta*180/M_PI <<
			" dth2:" << odo.delta_theta_2*180/M_PI <<
			" th:" << odo.theta*180/M_PI <<

			" acc: " << dbg_acc <<

			"");
	}

	odo.encWheelValue = 0;
	publishPose();
	calcJointStates();
	if (odo.p.publish_js) {
		publishJointStates();
	}
	if (calcTransform()) {
		if (odo.p.publish_tf) {
			publishTF();
		}
		publishOdometry();
	}
	publishVelocity();
}

void WheelOdometry::calcJointStates() {
	joint_state.header.stamp = odo.current_time; //ros::Time::now();

	int position = 0;

	if (!odo.p.animation_only) {
		joint_state.position[position++] += odo.dist_r_l / odo.p.wheel_ray;
		joint_state.position[position++] += odo.dist_r_r / odo.p.wheel_ray;
		joint_state.position[position++] += odo.dist_f_l / odo.p.wheel_ray;
		joint_state.position[position++] += odo.dist_f_r / odo.p.wheel_ray;

		//keep into 2*PI
		//fmod(joint_state.position[0], (2.0 * M_PI));
		//fmod(joint_state.position[1], (2.0 * M_PI));
		//fmod(joint_state.position[2], (2.0 * M_PI));
		//fmod(joint_state.position[3], (2.0 * M_PI));

		joint_state.position[position++] = odo.phi_l;
		joint_state.position[position++] = odo.phi_r;
	}
	joint_state.position[position++] = -(odo.encSteerValue / STEER_ENCODER_RESOLUTION) * (2 * M_PI);
}

void WheelOdometry::publishPose() {
	current_pose.x = odo.x; //base_to_odom_msg.transform.translation.x;
	current_pose.y = odo.y; //base_to_odom_msg.transform.translation.y;
	current_pose.theta = odo.theta; //tf::getYaw(odom.pose.pose.orientation);
	if(pose_pub.getNumSubscribers()>0) {
		pose_pub.publish(current_pose);
	}

	current_posestamped.header.stamp = odo.current_time;
	current_posestamped.pose.position.x = odo.x;
	current_posestamped.pose.position.y = odo.y;
	current_posestamped.pose.orientation.z = odo.theta;
	if(posestamped_pub.getNumSubscribers()>0) {
		posestamped_pub.publish(current_posestamped);
	}
}

void WheelOdometry::publishOdometry() {
	//next, we'll publish the odometry message over ROS
	odom.header.stamp = odo.current_time; //ros::Time::now();

	//set the position
	/*
	odom.pose.pose.position.x = base_to_odom_msg.transform.translation.x;
	odom.pose.pose.position.y = base_to_odom_msg.transform.translation.y;
	odom.pose.pose.position.z = base_to_odom_msg.transform.translation.z;
	odom.pose.pose.orientation.w = base_to_odom_msg.transform.rotation.w;
	odom.pose.pose.orientation.x = base_to_odom_msg.transform.rotation.x;
	odom.pose.pose.orientation.y = base_to_odom_msg.transform.rotation.y;
	odom.pose.pose.orientation.z = base_to_odom_msg.transform.rotation.z;
	*/

	//set the velocity
	/*
	tf::Transform orientation(
			tf::Quaternion(
					base_to_odom_msg.transform.rotation.x,
					base_to_odom_msg.transform.rotation.y,
					base_to_odom_msg.transform.rotation.z,
					base_to_odom_msg.transform.rotation.w));
	*/
	tf::Quaternion rotation = base_to_odom_msg.getRotation();
	tf::Vector3 translation = base_to_odom_msg.getOrigin();
	tf::Transform orientation(rotation);

	odom.pose.pose.position.x = translation.x();
	odom.pose.pose.position.y = translation.y();
	odom.pose.pose.position.z = translation.z();
	odom.pose.pose.orientation.w = rotation.w();
	odom.pose.pose.orientation.x = rotation.x();
	odom.pose.pose.orientation.y = rotation.y();
	odom.pose.pose.orientation.z = rotation.z();

	tf::Vector3 vel(0.0, 0.0, 0.0);
	if (odo.dt != 0) {
		vel.setX(odo.delta_x / odo.dt);
		vel.setY(odo.delta_y / odo.dt);
	};

	vel = orientation.inverse() * vel;
	odom.twist.twist.linear.x = vel.x();
	odom.twist.twist.linear.y = vel.y();
	odom.twist.twist.linear.z = vel.z();

	odom.twist.twist.angular.x = 0;
	odom.twist.twist.angular.y = 0;
	odom.twist.twist.angular.z = 0;
	if (odo.dt != 0) {
		odom.twist.twist.angular.z = odo.delta_theta / odo.dt;
	}

	odom_pub.publish(odom);
}

bool WheelOdometry::calcTransform() {
	//ros::Time current_time = odo.current_time;
	//ros::Time current_time = odo.current_time - ros::Duration(0.5); //past publish by 0.5
	ros::Time current_time = odo.current_time - ros::Duration(0.05); //slight delay

	try {
		listener.waitForTransform(
			odo.p.base_footprint,
			odo.p.base_odometry,
			ros::Time(0),
			ros::Duration(5.0));
	} catch (tf::TransformException &ex) {
			ROS_ERROR("lrm_odometry wait: %s", ex.what());
			return false;
	}

	/*publish transformed*/
	//desnecessario se o base_footprint estiver no meio do eixo traseiro
	//fixed transform
	tf::StampedTransform trans_base_enc;
	try {
		listener.lookupTransform(
				odo.p.base_footprint,
				odo.p.base_odometry,
				ros::Time(0),
				trans_base_enc);
	} catch (tf::TransformException &ex) {
		ROS_ERROR("lrm_odometry: %s", ex.what());
		return false;
	}

	/*
	std::cout <<
			"x:" << trans_base_enc.getOrigin().x() <<
			"y:" << trans_base_enc.getOrigin().y() <<
			"z:" << trans_base_enc.getOrigin().z() <<
			"rx:" << trans_base_enc.getRotation().x() <<
			"ry:" << trans_base_enc.getRotation().y() <<
			"rz:" << trans_base_enc.getRotation().z() <<
			"rw:" << trans_base_enc.getRotation().w() <<
			std::endl;
	*/

	tf::Quaternion qt = tf::createQuaternionFromYaw(odo.theta);

	tf::Transform trans_odom_encoder(
			qt,
			tf::Vector3(odo.x, odo.y, 0.0));

	tf::StampedTransform trans_odom_base_st(
			trans_odom_encoder,
			current_time, // - ros::Duration(0.5), /*trans_base_enc.stamp_,*/
			odo.p.fixed_odometry,
			odo.p.base_footprint);

	trans_odom_base_st *= trans_base_enc.inverse();
	//tf::transformStampedTFToMsg(trans_odom_base_st, base_to_odom_msg);
	//base_to_odom_msg.header.stamp = current_time;
	base_to_odom_msg = trans_odom_base_st;

	return true;
}

void WheelOdometry::publishTF() {
	odom_broadcaster.sendTransform(base_to_odom_msg);
}

void WheelOdometry::publishJointStates() {
	joint_pub.publish(joint_state);
}

void WheelOdometry::publishVelocity() {
	if(velocity_pub.getNumSubscribers()>0) {
		lrm_msgs::Velocity msg;
		msg.header.stamp = odo.current_time;
		//msg.header.frame_id = "base_footprint"
		msg.value = odo.v;
		velocity_pub.publish(msg);
	}
}


//void WheelOdometry::odomCallback(const ros::TimerEvent& t)
//{
//publishTF();
//publishJointStates();
//}

void WheelOdometry::resetPose()
{
	setPose(0, 0, 0);
}

void WheelOdometry::setPose(double x, double y, double theta)
{
	boost::unique_lock<boost::mutex> scoped_lock(mutex);
	odo.x = x;
	odo.y = y;
	odo.theta = theta;
}