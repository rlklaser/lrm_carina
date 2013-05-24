/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita on 27/08/2012
*********************************************************************/

/**
 * ADAPTACAO:
 *
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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <visualization_msgs/Marker.h>

#include "lrm_rbb_grupo_c/Goal.h"

// Simple Navigation States
typedef enum _SimpleNavigationState {
    
    SN_STOPPED = 1,
    SN_MOVING = 2,
    SN_ROTATING = 3,
    SN_MOVING_AS = 4,
    SN_ROTATING_AS = 5
    
} SimpleNavigationState;

// Simple Navigation State
SimpleNavigationState state;

// Global frame_id
std::string global_frame_id;
std::string odom_frame_id;

// Target position
geometry_msgs::PoseStamped goal;
// Robot odometry
nav_msgs::Odometry odom;
ros::Publisher marker_pub;
ros::Publisher reached_pub;

bool rotate_in_place;
double goal_tolerance;
int my_id;
int goal_id;

double my_goal_x;
double my_goal_y;

void displayGoal(double goal_x, double goal_y, bool reached)
{
	visualization_msgs::Marker goal_marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	goal_marker.header.frame_id =  reached ? global_frame_id : odom_frame_id;
	goal_marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	std::ostringstream ns;
	ns << "vehicle/" << my_id << "/goals";
	goal_marker.ns = "goals";
	goal_marker.id = goal_id;
	//goal_marker.lifetime

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	goal_marker.type = visualization_msgs::Marker::CYLINDER;

	// Set the marker action.  Options are ADD and DELETE
	goal_marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	goal_marker.pose.position.x = reached ? 0 : goal_x;
	goal_marker.pose.position.y = reached ? 0 : goal_y;
	goal_marker.pose.position.z = 0.0;
	goal_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

	// Set the scale of the marker
	goal_marker.scale.x = goal_tolerance*2.0;
	goal_marker.scale.y = goal_tolerance*2.0;
	goal_marker.scale.z = !reached ? 0.1 : 0.2;

	// Set the color -- be sure to set alpha to something non-zero!
	goal_marker.color.r = (!reached) ? 1.0 : 0.0;
	goal_marker.color.g = (!reached) ? 0.5 : 1.0;
	goal_marker.color.b = 0.0f;
	goal_marker.color.a = 0.5;

	goal_marker.lifetime = ros::Duration();

	// Publish the marker
	marker_pub.publish(goal_marker);
}

void goalReceived(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = *msg;

    ROS_INFO("goal received");

    if(rotate_in_place) state = SN_ROTATING;
    else state = SN_MOVING;

    my_goal_x = goal.pose.position.x;
    my_goal_y = goal.pose.position.y;

    displayGoal(goal.pose.position.x, goal.pose.position.y, false);
}

void odomReceived(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base_simple");
    
    ROS_INFO("Move Base Simple for ROS");
    
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    
    // Parameters
    double rate;
    double in_place_angular_velocity;
    double max_linear_velocity;
    double min_linear_velocity;
    double alpha;
    double attraction_coefficient;
    //double repulsion_coefficient;
    //double goal_tolerance;
    double angular_threshold;

    bool visualization;
    
    pn.param("rate", rate, 3.0);
    pn.param("in_place_angular_velocity", in_place_angular_velocity, 3.0);
    pn.param("max_linear_velocity", max_linear_velocity, 0.2);
    pn.param("min_linear_velocity", min_linear_velocity, 0.05);
    pn.param("alpha", alpha, 0.5);
    pn.param("attraction_coefficient", attraction_coefficient, 0.5);
    pn.param("goal_tolerance", goal_tolerance, 0.10);
    pn.param("angular_threshold", angular_threshold, 0.4);
    pn.param("visualization", visualization, false);
    pn.param("my_id", my_id, 0);

    if(angular_threshold == 0.0)
    {
		rotate_in_place = false;
		ROS_INFO("MoveBase Simple -- Not using in-place rotations.");
    }
    else
    {
    	rotate_in_place = true;
    	ROS_INFO("MoveBase Simple -- Using in-place rotations.");
    }

    pn.param<std::string>("global_frame_id", global_frame_id, "/base_link");
    pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
    
    ROS_INFO("MoveBase Simple -- Using %s as the global frame.", global_frame_id.c_str());
	
    // Making all the publishing and subscriptions...
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>(n.getNamespace() + "/cmd_vel", 10);
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>(n.getNamespace() + "/odom", 20, odomReceived);
    ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PoseStamped>(n.getNamespace() + "/move_base_simple/goal", 10, goalReceived);
    reached_pub= n.advertise<lrm_rbb_grupo_c::Goal>(n.getNamespace() + "/move_base_simple/reached", 2);
    marker_pub = n.advertise<visualization_msgs::Marker>(n.getNamespace() + "/simple_navigation_markers", 10);
    
    state = SN_STOPPED;
    goal_id = 0;
    
    // Main Loop
    ros::Rate r(rate);
    while(n.ok())
    {
        double linear_velocity = 0.0;
        double angular_velocity = 0.0;
        
        double current_orientation = tf::getYaw(odom.pose.pose.orientation);
        double current_x = odom.pose.pose.position.x;
        double current_y = odom.pose.pose.position.y;
        
        // If we reached our target position
        if((state == SN_MOVING || state == SN_MOVING_AS || state == SN_ROTATING || state == SN_ROTATING_AS) && sqrt(pow(current_x-goal.pose.position.x,2)+pow(current_y-goal.pose.position.y,2)) < goal_tolerance)
        {
            state = SN_STOPPED;
            linear_velocity = 0.0;
            angular_velocity = 0.0;
            ROS_WARN("REACHED!");
            displayGoal(my_goal_x, my_goal_y, true);
            goal_id++;

            lrm_rbb_grupo_c::Goal msg;
            msg.header.stamp = ros::Time::now();
            msg.reached = 1;
            msg.pose.x = current_x;
            msg.pose.y = current_y;
            msg.pose.theta = current_orientation;
            reached_pub.publish(msg);
        }

        // If we are moving...
        if(state == SN_MOVING || state == SN_MOVING_AS || state == SN_ROTATING || state == SN_ROTATING_AS)
        {
            double G_attr_x = -attraction_coefficient*(current_x-goal.pose.position.x);
            double G_attr_y = -attraction_coefficient*(current_y-goal.pose.position.y);
            
            double target_orientation = atan2(G_attr_y, G_attr_x);
            
            linear_velocity = sqrt(G_attr_x*G_attr_x + G_attr_y*G_attr_y);
            if(fabs(linear_velocity) > max_linear_velocity) linear_velocity = (linear_velocity > 0 ? max_linear_velocity : -max_linear_velocity);
            if(fabs(linear_velocity) < min_linear_velocity) linear_velocity = (linear_velocity > 0 ? min_linear_velocity : -min_linear_velocity);

			angular_velocity = -alpha*(angles::shortest_angular_distance(target_orientation, current_orientation));
	
			// If we intend to rotate before moving forward...
			if(state == SN_ROTATING || state == SN_ROTATING_AS)
			{
				linear_velocity = 0.0;
				angular_velocity = (angular_velocity < 0 ? -in_place_angular_velocity : in_place_angular_velocity);

				if(fabs(angles::shortest_angular_distance(current_orientation, target_orientation)) < angular_threshold)
				{
					angular_velocity = 0.0;
					if(state == SN_ROTATING) state = SN_MOVING;
					else if(state == SN_ROTATING_AS) state = SN_MOVING_AS;
				}
			}
        }
        
        // Send the new velocities to the robot...
        geometry_msgs::Twist cmd_vel;
        
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = angular_velocity;
        
        cmd_vel_pub.publish(cmd_vel);
        
        ros::spinOnce();
        r.sleep();
    }

  	return(0);
}

// EOF

