#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <atuacao/SteeringAngle.h>
#include <atuacao/Throttle.h>
#include <atuacao/Brake.h>
#include <controle/Velocity.h>
#include <queue>
#include <string>

#include <visualization_msgs/Marker.h>

#include "cGPSNavigation.h"

using namespace std;

//params
bool p_steeringActive = false;
bool p_throttleActive = false;
bool p_velocityActive = false;
int p_startDelay = 0;
double p_initialVelocity = 0;
double p_steeringFactor = 0.0;
double p_maxSteering = 0.0;
double p_minDist = 0.0;
double p_distError = 2.0;
double p_waypointTTL = 1.0;
double p_waypointHZ = 1.0;
double p_startX = 0.0;
double p_startY = 0.0;

typedef struct {
	double x;
	double y;
	double v;
	ros::Time ttl;
	long id;
} PointV;

atuacao::SteeringAngle _steer;
atuacao::Throttle _throttle;
controle::Velocity _velocity;
cGPSNavigation* _navi_current;
cGPSNavigation* _navi_next;
geometry_msgs::PoseStampedPtr _current_pose;
double _current_velocity;
PointV _current_point;
queue<PointV> _gps_points;
ros::Publisher _steering_pub;
ros::Publisher _throttle_pub;
ros::Publisher _velocity_pub;
ros::Time _begin;
ros::Time _last;
double _dist;
long _current_id;
ros::Publisher marker_pub;

double _relative_x;
double _relative_y;
double _first_point;

void add_marker(PointV& point) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "gps_marker";
	marker.id = point.id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.color.a = 0.8;
	marker.color.r = 1.0;
	marker.color.g = 0.5;
	marker.color.b = 0.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.24;
	marker.scale.z = 0.02;

	//marker.text = point.id;
	marker.pose.position.x = point.x;// - _relative_x;
	marker.pose.position.y = point.y;// - _relative_y;
	marker.pose.position.z = 2.5;

	marker_pub.publish(marker);
}

void remove_marker(PointV& point) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "gps_marker";
	marker.id = point.id;
	marker.action = visualization_msgs::Marker::DELETE;

	marker_pub.publish(marker);
}

PointV new_point(double x, double y, ros::Time ttl) {
	PointV pt;

	pt.ttl = ttl;
	pt.x = x;
	pt.y = y;
	pt.v = _current_velocity;
	pt.id = _current_id++;

	_gps_points.push(pt);

	ROS_WARN("new way point %f", ttl.toSec());

	add_marker(pt);

	return pt;
}

void callback_velocity(const std_msgs::Float32Ptr msgs) {
	_current_velocity = msgs->data;
//	if (_current_pose != 0) {
//		new_point(_current_pose->pose.position.x, _current_pose->pose.position.y, (ros::Time::now() + ros::Duration(p_waypointTTL)));
//	}
//
//	ROS_INFO("remote command, waypoint created - velocity set = %f", _current_velocity);
}

void callback_their(const geometry_msgs::PoseStampedPtr pose) {

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

void callback_our(const geometry_msgs::PoseStampedPtr pose) {
	double relativeAngleCurrent = _navi_current->getDirection(pose);
	double relativeAngleProx = _navi_next->getDirection(pose);

	ROS_WARN("Goal (%f ,%f); Pose (%f ,%f); Relative (%f, %f)", _navi_current->getGoalX(), _navi_current->getGoalY(), pose->pose.position.x, pose->pose.position.y, _navi_current->getRelativeX(), _navi_current->getRelativeY());
	ROS_WARN("Angle: %f  Dist angle Cur: %f Relative angle cur: %f Dist angle Prox: %f Relative angle prox: %f", _navi_current->getCurrentAngle(), _navi_current->getAngleToGoal(), relativeAngleCurrent, _navi_next->getAngleToGoal(), relativeAngleProx);

	//ros::Time now = pose->header.stamp;
	ros::Time now = ros::Time::now();

	bool expired = _current_point.ttl < now;
	//int throttle = _current_point.v;

	if (_navi_current->checkGoal(pose) || expired) {
		ROS_INFO("goal reached %f : (%s)", _current_point.ttl.toSec(), expired ? "expired" : "on time");

		if (!expired) {
			_navi_current->setGoal(_current_point.x, _current_point.y);
		} else {
			_navi_current->setGoal(pose->pose.position.x, pose->pose.position.y);
		}

		do {
			if (_gps_points.empty()) {
				ROS_WARN("no next point!");
				bool wait_steering = now < _begin + ros::Duration(p_startDelay);
				if (!wait_steering)
					_current_velocity = 0;
				break;
			}
			_current_point = _gps_points.front();
			_gps_points.pop();
			remove_marker(_current_point);
			expired = _current_point.ttl < now;

			ROS_INFO("next point %f (%f,%f) : (%s)", _current_point.ttl.toSec(), _current_point.x, _current_point.y, expired ? "expired, flushing..." : "on time");




		} while (expired); //consume all expired points

		if (p_throttleActive) {
			_throttle.value = _current_velocity;
			_throttle.header.frame_id = "/throttle";
			_throttle.header.stamp = now;
			_throttle_pub.publish(_throttle);

			ROS_INFO("throtle %f", _throttle.value);
		}

		if (p_velocityActive) {
			_velocity.value = _current_velocity;
			_velocity.header.frame_id = "/velocity";
			_velocity.header.stamp = now;
			_velocity_pub.publish(_velocity);

			ROS_INFO("velocity %f", _velocity.value);
		}

		_navi_next->setGoal(_current_point.x, _current_point.y);
	} else {

		if (p_steeringActive) {

			bool wait_steering = now < _begin + ros::Duration(p_startDelay);

			if (wait_steering) {
				ROS_WARN("steering delayed...");
			} else if (_current_velocity != 0) {

				double relativeAngle;
				_dist = sqrt(_navi_current->getRelativeX() * _navi_current->getRelativeX() + _navi_current->getRelativeY() * _navi_current->getRelativeY());

				if (_dist < p_minDist) {
					ROS_WARN("min dist");
					relativeAngle = (relativeAngleCurrent + relativeAngleProx) / 2;
				} else {
					relativeAngle = relativeAngleCurrent;
				}

				double steering = relativeAngle * p_steeringFactor;

				if (steering > p_maxSteering)
					steering = p_maxSteering;
				if (steering < -p_maxSteering)
					steering = -p_maxSteering;

				_steer.angle = steering * -1.0;
				_steer.header.frame_id = "/steering";
				_steer.header.stamp = now;
				_steering_pub.publish(_steer);

				ROS_INFO("angle %f dist %f", _steer.angle, _dist);
			} else {
				//_current_velocity = p_initialVelocity;
				ROS_WARN("Steering active but waiting for initial velocity...");
			}
		}

	}

}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "GPSFollowMe");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	nh_priv.param("distError", p_distError, 2.0);
	nh_priv.param("initialVelocity", p_initialVelocity, 1.0);
	nh_priv.param("maxSteering", p_maxSteering, 25.0);
	nh_priv.param("minDist", p_minDist, 1.0);
	nh_priv.param("startDelay", p_startDelay, 10);
	nh_priv.param("startX", p_startX, 0.0);
	nh_priv.param("startY", p_startY, 0.0);
	nh_priv.param("steer", p_steeringActive, true);
	nh_priv.param("velocity", p_velocityActive, true);
	nh_priv.param("throttle", p_throttleActive, false);
	nh_priv.param("steeringFactor", p_steeringFactor, 0.5);
	nh_priv.param("waypointHZ", p_waypointHZ, 1.0);
	nh_priv.param("waypointTTL", p_waypointTTL, 3.0);

	_begin = ros::Time::now();
	_last = _begin;
	_current_velocity = p_initialVelocity;
	_current_id = 0;

	_first_point = true;
	_relative_x = 0;
	_relative_y = 0;

	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	_current_point = new_point(p_startX, p_startY, _begin);
	_gps_points.pop();

	_navi_current = new cGPSNavigation(0, 0, p_distError, cGPSNavigation::MODE_AUTO);
	_navi_next = new cGPSNavigation(0, 0, p_distError, cGPSNavigation::MODE_AUTO);

	ros::Subscriber subPoseA = nh.subscribe("followme/their/pose", 1, callback_their);
	ros::Subscriber subPoseB = nh.subscribe("followme/our/pose", 1, callback_our);
	ros::Subscriber subVel = nh.subscribe("followme/velocity", 1, callback_velocity);

	if (p_steeringActive) {
		_steer.angle = 0;
		_steering_pub = nh.advertise<atuacao::SteeringAngle>("followme/steering_command", 1);
		ROS_INFO("will send steer command");
	}

	if (p_throttleActive) {
		_throttle.value = 0;
		_throttle_pub = nh.advertise<atuacao::Throttle>("followme/throttle_command", 1);
		ROS_INFO("will send throttle command");
	}

	if (p_velocityActive) {
		_velocity.value = 0;
		_velocity_pub = nh.advertise<controle::Velocity>("followme/velocity_command", 1);
		ROS_INFO("will send velocity command");
	}

	ROS_INFO("Start spinning...");

	ros::spin();

	return 0;
}
