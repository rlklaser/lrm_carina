/*
 * Copyright (c) 2012, Siddhant Ahuja (Sid)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Siddhant Ahuja (Sid) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Desc: Gazebo 1.x plugin for a Ackermann Drive Simulation Robot
 * Adapted from the Erratic and Turtlebot Robot plugin
 * Author: Siddhant Ahuja
 */

#ifndef ACKERMANN_PLUGIN_HH
#define ACKERMANN_PLUGIN_HH

#include <map>

#include <common/common.h>
#include <physics/physics.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <lrm_msgs/RobotStates.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {
class Joint;
class Entity;

class AckermannPlugin: public ModelPlugin {
public:
	AckermannPlugin();
	~AckermannPlugin();
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
	virtual void UpdateChild();
	virtual void FiniChild();

private:
	void write_position_data();
	void populateCovariance(nav_msgs::Odometry &msg);
	void publish_wheel_odometry(double stAngle);
	void GetPositionCmd();

	physics::WorldPtr world;
	physics::ModelPtr parent;
	event::ConnectionPtr updateConnection;

	std::string backLeftJointName;
	std::string backRightJointName;
	std::string frontSteerJointName;

	//double wheelBase;
	//double wheelTrack;
	double wheelDiameter;
	double driveTorque;
	double steerTorque;
	double steerAngle;
	double maxSteerAngle;
	bool velControl;
	double turnAngle;
	double diffAngle;
	double wheelSpeed[3];
	double wheelOdomPose[3];
	double wheelOdomVel[3];

	physics::JointPtr joints[3];
	physics::PhysicsEnginePtr physicsEngine;

	// ROS STUFF
	ros::NodeHandle* rosnode_;
	ros::Publisher pub_;
	ros::Publisher pub_states_;
	ros::Subscriber sub_;
	tf::TransformBroadcaster *transform_broadcaster_;
	nav_msgs::Odometry wheel_odom_;
	std::string tf_prefix_;
	lrm_msgs::RobotStates msg_states_;

	boost::mutex lock;

	std::string robotNamespace;
	std::string topicName;

	// Custom Callback Queue
	ros::CallbackQueue queue_;
	boost::thread callback_queue_thread_;
	void QueueThread();

	// AckermannDrive stuff
	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

	double x_;
	double rot_;
	bool alive_;

	double sigma_x_;
	double sigma_y_;
	double sigma_theta_;
	double cov_x_y_;
	double cov_x_theta_;
	double cov_y_theta_;
};

}

#endif
