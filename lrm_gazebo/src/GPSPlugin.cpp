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
 * @file GPSPlugin.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Dec 15, 2012
 *
 */

#include <sensor_msgs/NavSatStatus.h>

#include "GPSPlugin.h"

namespace gazebo
{

GPSPlugin::GPSPlugin()
{
	   , position_error_model_(parameters)
	   , velocity_error_model_(parameters, "velocity")

	// Start up ROS
	std::string name = "ros_gps_plugin_node";
	int argc = 0;
	ros::init(argc, NULL, name);

	fix_publisher_ = node_handle_->advertise<sensor_msgs::NavSatFix>(**fix_topic_, 10);
	  velocity_publisher_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(**velocity_topic_, 10);
}

GPSPlugin::~GPSPlugin()
{
	delete _node;
}

void GPSPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
	_model = parent;
	_node = new ros::NodeHandle("~");

	if (!sdf->HasElement("latitude"))
	{
		ROS_WARN("GPS plugin missing <latitude>, defaults to 00.00");
		_latitude = 0;
	}
	else
	{
		_latitude = sdf->GetElement("latitude")->GetValueDouble();
	}

	if (!sdf->HasElement("longitude"))
	{
		ROS_WARN("GPS plugin missing <longitude>, defaults to 00.00");
		_longitude = 0;
	}
	else
	{
		_longitude = sdf->GetElement("longitude")->GetValueDouble();
	}

	if (!sdf->HasElement("elevation"))
	{
		ROS_WARN("GPS plugin missing <elevation>, defaults to 00.00");
		_elevation = 0;
	}
	else
	{
		_elevation = sdf->GetElement("elevation")->GetValueDouble();
	}


	  Param::Begin(&parameters);
	  namespace_ = new ParamT<std::string>("robotNamespace", "", false);
	  body_name_ = new ParamT<std::string>("bodyName", "", true);
	  frame_id_ = new ParamT<std::string>("frameId", "", false);
	  fix_topic_ = new ParamT<std::string>("topicName", "fix", false);
	  velocity_topic_ = new ParamT<std::string>("velocityTopicName", "fix_velocity", false);

	  reference_latitude_ = new ParamT<double>("referenceLatitude", 49.9, false);
	  reference_longitude_ = new ParamT<double>("referenceLongitude", 8.9, false);
	  reference_heading_ = new ParamT<double>("referenceHeading", 0.0, false);
	  reference_altitude_ = new ParamT<double>("referenceAltitude", 0.0, false);
	  status_ = new ParamT<sensor_msgs::NavSatStatus::_status_type>("status", sensor_msgs::NavSatStatus::STATUS_FIX, false);
	  service_ = new ParamT<sensor_msgs::NavSatStatus::_service_type>("service", sensor_msgs::NavSatStatus::SERVICE_GPS, false);
	  Param::End();

	// ROS Subscriber
	//_sub = _node->subscribe<std_msgs::Float64>("x", 1000, &ROSModelPlugin::ROSCallback, this);
	/*
	 UTM::LLtoUTM(origin_lat_, origin_long_,
	 origin_northing_, origin_easting_, origin_zone_);
	 ROS_INFO("Map origin UTM: %s, %.2f, %.2f",
	 origin_zone_, origin_easting_, origin_northing_);

	 // Round UTM origin of map to nearest UTM grid intersection.
	 double grid_x = (rint(origin_easting_/UTM::grid_size) * UTM::grid_size);
	 double grid_y = (rint(origin_northing_/UTM::grid_size) * UTM::grid_size);
	 ROS_INFO("UTM grid origin: (%.f, %.f)", grid_x , grid_y);

	 // Report stage-generated odometry relative to that location.
	 map_origin_x_ = origin_easting_ - grid_x;
	 map_origin_y_ = origin_northing_ - grid_y;
	 ROS_INFO("MapXY origin: (%.f, %.f)", map_origin_x_ , map_origin_y_);
	 }
	 */
	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	//_updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&GPSPlugin::OnUpdate, this));
	_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GPSPlugin::OnUpdate, this));
}

// Called by the world update start event
void GPSPlugin::OnUpdate()
{
	ros::spinOnce();


	  Time sim_time = Simulator::Instance()->GetSimTime();
	  double dt = (sim_time - lastUpdate).Double();

	  Pose3d pose = body_->GetWorldPose();

	  gazebo::Vector3 velocity = velocity_error_model_(body_->GetWorldLinearVel(), dt);
	  position_error_model_.setCurrentDrift(position_error_model_.getCurrentDrift() + velocity_error_model_.getCurrentError() * dt);
	  gazebo::Vector3 position = position_error_model_(pose.pos, dt);

	  fix_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
	  velocity_.header.stamp = fix_.header.stamp;

	  fix_.latitude  = **reference_latitude_  + ( cos(**reference_heading_) * position.x + sin(**reference_heading_) * position.y) / EARTH_RADIUS * 180.0/M_PI;
	  fix_.longitude = **reference_longitude_ - (-sin(**reference_heading_) * position.x + cos(**reference_heading_) * position.y) / EARTH_RADIUS * 180.0/M_PI * cos(fix_.latitude);
	  fix_.altitude  = **reference_altitude_  + position.z;
	  fix_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
	  velocity_.vector.x =  cos(**reference_heading_) * velocity.x + sin(**reference_heading_) * velocity.y;
	  velocity_.vector.y = -sin(**reference_heading_) * velocity.x + cos(**reference_heading_) * velocity.y;
	  velocity_.vector.z = velocity.z;

	  fix_publisher_.publish(fix_);
	  velocity_publisher_.publish(velocity_);
}

void GPSPlugin::publishGPS(ros::Time sim_time)
{
	art_msgs::GpsInfo gpsi;

	gpsi.header.stamp = sim_time;
	gpsi.header.frame_id = tf_prefix_ + ArtFrames::odom;

	// relocate pose relative to map origin
	gpsi.utm_e = (odomMsg_.pose.pose.position.x - map_origin_x_ + origin_easting_);
	gpsi.utm_n = (odomMsg_.pose.pose.position.y - map_origin_y_ + origin_northing_);

	UTM::UTMtoLL(gpsi.utm_n, gpsi.utm_e, origin_zone_, gpsi.latitude, gpsi.longitude);

	gpsi.zone = origin_zone_;
	gpsi.altitude = odomMsg_.pose.pose.position.z;
	gpsi.quality = art_msgs::GpsInfo::DGPS_FIX;
	gpsi.num_sats = 9;

	gps_pub_.publish(gpsi);
}

//void ROSCallback(const std_msgs::Float64::ConstPtr& msg)
//{
//  ROS_INFO("subscriber got: [%f]", msg->data);
//}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN (GPSPlugin)
}

