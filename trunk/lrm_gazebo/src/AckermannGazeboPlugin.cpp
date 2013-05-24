#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <ros/ros.h>

namespace gazebo {

enum {
	RIGHT, LEFT, STEER
};

const double TAU = 6.28318530717958647693; // 2 * pi

class AckermannGazeboPlugin: public ModelPlugin {

public:
	AckermannGazeboPlugin() {

		// Start up ROS
		std::string name = "ackerman_plugin";
		int argc = 0;
		ros::init(argc, NULL, name);
	}

	~AckermannGazeboPlugin() {
		delete node_;
		  delete callback_queue_thread_;
		  delete rosnode_;
		  delete transform_broadcaster_;

	}

	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
		// Store the pointer to the model
		model_ = _parent;

		// ROS Nodehandle
		node_ = new ros::NodeHandle("~");

		// ROS Subscriber
//		sub_ = this->node->subscribe<std_msgs::Float64>("x", 1000, &ROSModelPlugin::ROSCallback, this);

		// Listen to the update event. This event is broadcast every
		// simulation iteration.
//		updateConnection_ = event::Events::ConnectWorldUpdateStart(boost::bind(&ROSModelPlugin::OnUpdate, this));


		enableMotors = true;

		wheelSpeed[RIGHT] = 0;
		wheelSpeed[LEFT] = 0;
		steerAngle = 0;

		//prevUpdateTime = Simulator::Instance()->GetSimTime();
		leftJointName = _sdf->GetValueString("leftJoint");
		rightJointName = _sdf->GetValueString("rightJoint");
		steerJointName = _sdf->GetValueString("steerJoint");
		wheelSepWidthP =  _sdf->GetValueDouble("wheelSeparationWidth");
		wheelSepLengthP =  _sdf->GetValueDouble("wheelSeparationLength");
		wheelDiamP =  _sdf->GetValueDouble("wheelDiameter");
		driveTorqueP =  _sdf->GetValueDouble("driveTorque");
		steerTorqueP =  _sdf->GetValueDouble("steerTorque");
		robotNamespace = _sdf->GetValueString("robotNamespace");
		topicName = _sdf->GetValueString("topicName");

		speed_ = 0;
		angle_ = 0;
		alive_ = true;



		  // ROS: Subscribe to the CarDrive command topic (usually "car_drive")
		  ros::SubscribeOptions so =
		      ros::SubscribeOptions::create<art_msgs::CarDriveStamped>(topicName, 1,
		                                                               boost::bind(&AckermannPlugin::carDriveCallback, this, _1),
		                                                               ros::VoidPtr(), &queue_);
		  sub_ = rosnode_->subscribe(so);
		  pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);


		  //callback_queue_thread_ = new boost::thread(boost::bind(&AckermannPlugin::QueueThread, this));

	}

	// Called by the world update start event
	void OnUpdate() {
		ros::spinOnce();

		double wd, wsw, wsl;
		  double d1, d2, a;
		  double dr, da, r;
		  Time stepTime;

		  //myIface->Lock(1);

		  GetPositionCmd();

		  wd = **(wheelDiamP);
		  wsw = **(wheelSepWidthP);
		  wsl = **(wheelSepLengthP);

		  //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
		  stepTime = Simulator::Instance()->GetSimTime() - prevUpdateTime;
		  prevUpdateTime = Simulator::Instance()->GetSimTime();

		  // Distance traveled by each back wheel
		  d1 = stepTime.Double() * wd / 2 * joints[LEFT]->GetVelocity(0);
		  d2 = stepTime.Double() * wd / 2 * joints[RIGHT]->GetVelocity(0);

		  // Get steering angle
		  a = joints[STEER]->GetAngle(0).GetAsRadian();

		  // compute radius of curvature
		  // may be infinite. that's ok
		  r = wsl * tan(TAU/2 - a);

		  // compute displacement along arc
		  dr = (d1 + d2) / 2;
		  da = dr / r;

		  // Compute odometric pose
		  odomPose[0] += dr * cos(odomPose[2]);
		  odomPose[1] += dr * sin(odomPose[2]);
		  odomPose[2] += da;

		  // Compute odometric instantaneous velocity
		  odomVel[0] = dr / stepTime.Double();
		  odomVel[1] = 0.0;
		  odomVel[2] = da / stepTime.Double();

		  if (enableMotors)
		  {
		    joints[LEFT]->SetVelocity(0, wheelSpeed[LEFT] / (**(wheelDiamP) / 2.0));

		    joints[RIGHT]->SetVelocity(0, wheelSpeed[RIGHT] / (**(wheelDiamP) / 2.0));

		    // FIXME: come up with something nicer for doing position control
		    // than this simple proportional controller
		    joints[STEER]->SetVelocity(0, (steerAngle - a) / stepTime.Double());

		    joints[LEFT]->SetMaxForce(0, **(driveTorqueP));
		    joints[RIGHT]->SetMaxForce(0, **(driveTorqueP));
		    joints[STEER]->SetMaxForce(0, **(steerTorqueP));
		  }

		  write_position_data();
		  publish_odometry();

	}

	void ROSCallback(const std_msgs::Float64::ConstPtr& msg) {
		ROS_INFO("subscriber got: [%f]", msg->data);
	}

	// Pointer to the model
private:
	physics::ModelPtr model_;
	// Pointer to the update event connection
	event::ConnectionPtr updateConnection_;
	// ROS Nodehandle
	ros::NodeHandle* node_;

	// ROS Subscriber
	ros::Subscriber sub_;

	libgazebo::PositionIface *pos_iface_;
	Model *parent_;
	double wheelSepWidthP;
	double wheelSepLengthP;
	double wheelDiamP;
	double driveTorqueP;
	double steerTorqueP;
	float wheelSpeed[2];
	float steerAngle;

	// Simulation time of the last update
	ros::Time prevUpdateTime;

	bool enableMotors;
	float odomPose[3];
	float odomVel[3];

	Joint *joints[3];
	PhysicsEngine *physicsEngine;
	std::string leftJointName;
	std::string rightJointName;
	std::string steerJointName;

	// ROS STUFF
	ros::NodeHandle* rosnode_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
	tf::TransformBroadcaster *transform_broadcaster_;
	nav_msgs::Odometry odom_;
	std::string tf_prefix_;
	boost::mutex lock;
	std::string robotNamespace;
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

	void publish_odometry()
	{
	  // get current time
	  ros::Time current_time_((Simulator::Instance()->GetSimTime()).sec, (Simulator::Instance()->GetSimTime()).nsec);

	  // getting data for base_footprint to odom transform
	  btQuaternion qt;
	  // TODO: Is there something wrong here? RVIZ has a problem?
	  qt.setEulerZYX(pos_iface_->data->pose.yaw, pos_iface_->data->pose.pitch, pos_iface_->data->pose.roll);
	  btVector3 vt(pos_iface_->data->pose.pos.x, pos_iface_->data->pose.pos.y, pos_iface_->data->pose.pos.z);
	  tf::Transform base_footprint_to_odom(qt, vt);

	  transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
	                                                            current_time_,
	                                                            "odom",
	                                                            "base_footprint"));

	  // publish odom topic
	  odom_.pose.pose.position.x = pos_iface_->data->pose.pos.x;
	  odom_.pose.pose.position.y = pos_iface_->data->pose.pos.y;

	  gazebo::Quatern rot;
	  rot.SetFromEuler(gazebo::Vector3(pos_iface_->data->pose.roll, pos_iface_->data->pose.pitch, pos_iface_->data->pose.yaw));

	  odom_.pose.pose.orientation.x = rot.x;
	  odom_.pose.pose.orientation.y = rot.y;
	  odom_.pose.pose.orientation.z = rot.z;
	  odom_.pose.pose.orientation.w = rot.u;

	  odom_.twist.twist.linear.x = pos_iface_->data->velocity.pos.x;
	  odom_.twist.twist.linear.y = pos_iface_->data->velocity.pos.y;
	  odom_.twist.twist.angular.z = pos_iface_->data->velocity.yaw;

	  odom_.header.frame_id = tf::resolve(tf_prefix_, "odom");
	  odom_.child_frame_id = "base_footprint";

	  //odom_.header.stamp = current_time_;
	  odom_.header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
	  odom_.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;

	  pub_.publish(odom_);
	}

	// Update the data in the interface
	void write_position_data()
	{
	  // TODO: Data timestamp
	  pos_iface_->data->head.time = Simulator::Instance()->GetSimTime().Double();

	  pos_iface_->data->pose.pos.x = odomPose[0];
	  pos_iface_->data->pose.pos.y = odomPose[1];
	  pos_iface_->data->pose.yaw = NORMALIZE(odomPose[2]);

	  pos_iface_->data->velocity.pos.x = odomVel[0];
	  pos_iface_->data->velocity.yaw = odomVel[2];

	  // TODO
	  pos_iface_->data->stall = 0;
	}

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AckermannGazeboPlugin)
}
