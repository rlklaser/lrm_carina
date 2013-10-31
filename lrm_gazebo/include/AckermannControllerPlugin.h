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
 * @file AckermannControllerPlugin.h
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Dec 13, 2012
 *
 */

#ifndef ACKERMANNCONTROLLER_H_
#define ACKERMANNCONTROLLER_H_

#include <map>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>
//#include <gazebo/sdf/sdf.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <lrm_msgs/RobotStates.h>
#include <lrm_msgs/Encoders.h>

#include <lrm_msgs/Steering.h>
#include <lrm_msgs/Throttle.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <angles/angles.h>

namespace gazebo {

enum
{
	BACKRIGHT, BACKLEFT, FRONTSTEER, /*STEERWHEEL, WHEELENCODER,*/ NUM_JOINTS
};

class Joint;
class Entity;

class AckermannControllerPlugin: public ModelPlugin {
public:
	AckermannControllerPlugin();
	~AckermannControllerPlugin();
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
	virtual void UpdateChild();
	virtual void FiniChild();

private:
	void GetPositionCmd();

	physics::WorldPtr world_;
	physics::ModelPtr parent_;
	event::ConnectionPtr connection_;

	std::string backLeftJointName_;
	std::string backRightJointName_;
	std::string frontSteerJointName_;
	//std::string steeringWheelJointName_;
	//std::string wheelEncoderJointName_;

	double wheelDiameter_;
	double driveTorque_;
	double steerTorque_;
	double steerAngle_;
	double maxSteerAngle_;
	double turnAngle_;
	double diffAngle_;
	double wheelSpeed_[3];
	double maxThrottle_;

	physics::JointPtr joints_[NUM_JOINTS];
	physics::PhysicsEnginePtr physicsEngine_;

	// ROS STUFF
	ros::NodeHandle* rosnode_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
	ros::Subscriber sub_steer_;
	ros::Subscriber sub_throttle_;

	std::string tf_prefix_;

	boost::mutex lock_;

	std::string robotNamespace_;
	std::string topicName_;

	common::Time last_update_;
	int freq_;
	double accWheel_;

	double prevUpdateTime;

	// Custom Callback Queue
	ros::CallbackQueue queue_;
	boost::thread callback_queue_thread_;
	void QueueThread();

	// AckermannDrive stuff
	void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
	void SteeringCallback(const lrm_msgs::Steering::ConstPtr& cmd_msg);
	void ThrottleCallback(const lrm_msgs::Throttle::ConstPtr& cmd_msg);

	double x_;
	double rot_;
	bool alive_;

	bool _use_cmd_vel;

};

}

#endif /* ACKERMANNCONTROLLER_H_ */
