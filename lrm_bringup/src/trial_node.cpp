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
 * @file send_goal.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Apr 25, 2014
 *
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <lrm_msgs/VehicleState.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sbpl_lattice_planner/SBPLLatticePlannerStats.h>
#include <lrm_msgs/Encoders.h>
#include <lrm_msgs/Throttle.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>


// /move_base/SBPLLatticePlanner/plan					nav_msgs/Path
// /move_base/TrajectoryPlannerROS/local_plan			nav_msgs/Path
// /planning/cmd_vel									geometry_msgs/Twist
// /move_base/status     							 	actionlib_msgs/GoalStatusArray
// /wheel_odometry/odom  								nav_msgs/Odometry


bool start, stop;



void status_cb(actionlib_msgs::GoalStatusArray::ConstPtr msg)
{
	if(!start) return;
}

double old_x = 0;
double old_y = 0;
double dist = 0;

double global_count = 0;

void global_plan_cb(nav_msgs::Path::ConstPtr msg)
{
	if(!start) return;

	global_count++;
}

double local_count = 0;

void local_plan_cb(nav_msgs::Path::ConstPtr msg)
{
	if(!start) return;

	local_count++;
}

double control_count = 0;


void control_cb(geometry_msgs::Twist::ConstPtr msg)
{
	if(!start) return;

	control_count++;
}

double sum_vel = 0;

//void state_cb(lrm_msgs::VehicleStates::ConstPtr msg)
//{
//}

void odom_cb(nav_msgs::Odometry::ConstPtr msg)
{

	//std::cout << "got odom" << std::endl;

	double x = msg->pose.pose.position.x;
	double y = msg->pose.pose.position.y;

	double dx = x-old_x;
	double dy = y-old_y;

	old_x = x;
	old_y = y;

	if(!start) return;


	dist+= sqrt(dx*dx + dy*dy);

	std::cout << "Distance " << dist << " Tracks " << global_count << " Plans " << local_count << " Controls " << control_count << std::endl;

}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv){
  ros::init(argc, argv, "trial_node");

  ros::NodeHandle nh;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 55.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.position.z = 0;
  //tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 3*M_PI/4);
  //tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, M_PI);
  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
  goal.target_pose.pose.orientation.x = q.x();
  goal.target_pose.pose.orientation.y = q.y();
  goal.target_pose.pose.orientation.z = q.z();
  goal.target_pose.pose.orientation.w = q.w();


  ros::Subscriber sub_odom;
  ros::Subscriber sub_status;
  ros::Subscriber sub_local_plan;
  ros::Subscriber sub_global_plan;
  ros::Subscriber sub_control;


  sub_odom = nh.subscribe<nav_msgs::Odometry>("/wheel_odometry/odom", 1, &odom_cb);
  sub_status = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1, &status_cb);
  sub_control = nh.subscribe<geometry_msgs::Twist>("/planning/cmd_vel", 1, &control_cb);
  sub_local_plan = nh.subscribe<nav_msgs::Path>("/move_base/TrajectoryPlannerROS/local_plan", 1, &local_plan_cb);
  sub_global_plan = nh.subscribe<nav_msgs::Path>("/move_base/SBPLLatticePlanner/plan", 1, &global_plan_cb);


  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ros::Time start_time = ros::Time::now();


  start = true;

  //ac.waitForResult();

  //for(int i=0; i<1000000; i++)
  for(;;)
  {
	  ros::spinOnce();
	  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		  stop = true;
		  start = false;
		  ROS_INFO("Hooray, the base moved forward");
		  break;
	  }
  //else
  //  ROS_INFO("The base failed to move forward for some reason");
  }


  ros::Time stop_time = ros::Time::now();

  ros::Duration dur = stop_time - start_time;


  std::cout << "Total global plans: " << global_count << std::endl;
  std::cout << "Total local plans: " << local_count << std::endl;
  std::cout << "Total controls: " << control_count << std::endl;
  std::cout << "Total time (s) " <<  dur.toSec()  <<  " from: " << start_time << " to: " << stop_time << std::endl;
  std::cout << "Avg speed " << dist / dur.toSec() << std::endl;

  return 0;
}
