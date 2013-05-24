/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: ROS interface to a Position2d controller for a Differential drive.
 * Author: Daniel Hewlett (adapted from Nathan Koenig)
 */
/*
 * Adapted as a ROS gazebo plugin for car-like vehicles using
 * Ackermann steering.
 *
 * Copyright (C) 2011, Nicu Stiurca, Jack O'Quin
 */
#ifndef ACKERMANN_PLUGIN_H
#define ACKERMANN_PLUGIN_H

#include <map>

//#include <gazebo/Param.hh>
//#include <gazebo/Controller.hh>
//#include <gazebo/Model.hh>

// ROS 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//#include <art_msgs/CarDriveStamped.h>
#include <nav_msgs/Odometry.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{
class Joint;
class Entity;

class AckermannPlugin : public Controller
{

public:
  AckermannPlugin(Entity *parent);
  virtual ~AckermannPlugin();

protected:
  virtual void LoadChild(XMLConfigNode *node);
  void SaveChild(std::string &prefix, std::ostream &stream);
  virtual void InitChild();
  void ResetChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:
  void write_position_data();
  void publish_odometry();
  void GetPositionCmd();

  libgazebo::PositionIface *pos_iface_;
  Model *parent_;
  ParamT<float> *wheelSepWidthP;
  ParamT<float> *wheelSepLengthP;
  ParamT<float> *wheelDiamP;
  ParamT<float> *driveTorqueP;
  ParamT<float> *steerTorqueP;
  float wheelSpeed[2];
  float steerAngle;

  // Simulation time of the last update
  Time prevUpdateTime;

  bool enableMotors;
  float odomPose[3];
  float odomVel[3];

  Joint *joints[3];
  PhysicsEngine *physicsEngine;
  ParamT<std::string> *leftJointNameP;
  ParamT<std::string> *rightJointNameP;
  ParamT<std::string> *steerJointNameP;

  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  tf::TransformBroadcaster *transform_broadcaster_;
  nav_msgs::Odometry odom_;
  std::string tf_prefix_;

  boost::mutex lock;

  ParamT<std::string> *robotNamespaceP;
  std::string robotNamespace;

  ParamT<std::string> *topicNameP;
  std::string topicName;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread* callback_queue_thread_;
  void QueueThread();

  // Ackermann stuff
  void carDriveCallback(const art_msgs::CarDriveStamped::ConstPtr& cmd_msg);

  float speed_;
  float angle_;
  bool alive_;
};

}

#endif // ACKERMANN_PLUGIN_H
