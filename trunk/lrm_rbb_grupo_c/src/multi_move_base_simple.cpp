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
*  CAUSED AND ON ANY TH#include <angles/angles.h>EORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita on 03/09/2012
*********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <angles/angles.h>
#include <visualization_msgs/Marker.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

//typedefs to help us out with the action server so that we don't hace to type so much
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

MoveBaseActionServer * as;

ros::NodeHandle * n_ptr;
ros::Publisher * cmd_vel_pub_ptr;
ros::Publisher * marker_pub_ptr;

nav_msgs::OccupancyGrid map;

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

// tf
tf::TransformListener * tf_listener;

double distance_travelled;

// Parameters
std::string global_frame_id;
double rate;
double in_place_angular_velocity;
double max_linear_velocity;
double min_linear_velocity;
double alpha;
double attraction_coefficient;
double repulsion_coefficient_robots;
double repulsion_coefficient_obstacles;
double goal_tolerance;
double angular_threshold;
double repulsion_threshold_robots;
double repulsion_threshold_obstacles;
ros::Duration delay;
double max_distance;
ros::Duration goal_timeout;

bool visualization;

int simple_goal = 0;

// Robot data structure
class Robot
{
    public:
    Robot(int id) : pose(), prefix()
    {
        this->id = id;
        prefix = "/vehicle/";
        prefix.append<int>(1, 48+id);
    };
    
    Robot(const Robot& r) : pose(), prefix()
    {
        id = r.id;
        
        pose = r.pose;
        
        odom_sub = r.odom_sub;
        tf_filter = r.tf_filter;

	prefix = r.prefix;
    };
    
    // Robot ID
    int id;
    
    // Robot data
    geometry_msgs::PoseStamped pose;
    
    // Subscribers
    message_filters::Subscriber<nav_msgs::Odometry> * odom_sub;
    tf::MessageFilter<nav_msgs::Odometry> * tf_filter;

    std::string prefix;
};

// The robots
std::vector<Robot> robots;

int me;

bool rotate_in_place;

double calculateVelocities(double current_x, double current_y, double current_orientation, double goal_x, double goal_y, double * linear_velocity, double * angular_velocity)
{
	// Attraction
        double G_attr_x = -attraction_coefficient*(current_x - goal_x);
        double G_attr_y = -attraction_coefficient*(current_y - goal_y);
            
	// Repulsion
	double G_rep_x = 0.0;
	double G_rep_y = 0.0;

	// Robots
	for(int i=0 ; i<robots.size() ; i++)
	{
		if(i==me) continue;
		
		double distance = sqrt(pow(current_x-robots[i].pose.pose.position.x,2)+pow(current_y-robots[i].pose.pose.position.y,2));
		if(distance <= repulsion_threshold_robots)
		{
			G_rep_x += -repulsion_coefficient_robots*(current_x-robots[i].pose.pose.position.x)*(1/pow(distance,2)-repulsion_threshold_robots/pow(distance,3));
			G_rep_y += -repulsion_coefficient_robots*(current_y-robots[i].pose.pose.position.y)*(1/pow(distance,2)-repulsion_threshold_robots/pow(distance,3));
		}
	}

	// Obstacles on the map
	int current_x_index = current_x/map.info.resolution;
	int current_y_index = current_y/map.info.resolution;

	int rep = ceil(repulsion_threshold_obstacles/map.info.resolution)+1;
	for(int i=(current_x_index-rep<0 ? 0 : current_x_index-rep) ; i<(current_x_index+rep>map.info.width ? map.info.width : current_x_index+rep) ; i++)
	{
		for(int j=(current_y_index-rep<0 ? 0 : current_y_index-rep) ; j<(current_y_index+rep>map.info.height ? map.info.height : current_y_index+rep) ; j++)
		{
			double distance = sqrt(pow(current_x-i*map.info.resolution,2)+pow(current_y-j*map.info.resolution,2));
			if(map.data[j*map.info.width+i] != 0 && distance <= repulsion_threshold_obstacles)
			{
				G_rep_x += -repulsion_coefficient_obstacles*(current_x-i*map.info.resolution)*(1/pow(distance,2)-repulsion_threshold_obstacles/pow(distance,3));
				G_rep_y += -repulsion_coefficient_obstacles*(current_y-j*map.info.resolution)*(1/pow(distance,2)-repulsion_threshold_obstacles/pow(distance,3));
			}
		}
	}

	double G_x = G_attr_x + G_rep_x;
	double G_y = G_attr_y + G_rep_y;

        double target_orientation = atan2(G_y, G_x);
            
        *linear_velocity = sqrt(G_x*G_x + G_y*G_y);
	if(fabs(*linear_velocity) > max_linear_velocity) *linear_velocity = (*linear_velocity > 0 ? max_linear_velocity : -max_linear_velocity);
        if(fabs(*linear_velocity) < min_linear_velocity) *linear_velocity = (*linear_velocity > 0 ? min_linear_velocity : -min_linear_velocity);
            
	double delta_yaw = angles::shortest_angular_distance(target_orientation, current_orientation);

        *angular_velocity = -alpha*(delta_yaw);

	if(visualization)
	{
		visualization_msgs::Marker marker;
	    	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		std::string frame_id = robots[me].prefix;
		frame_id.append("/base_link");
	    	marker.header.frame_id = frame_id;
	    	marker.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "attraction_repulsion_vectors";
		marker.id = robots[me].id;

		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker.type = visualization_msgs::Marker::ARROW;

	 	// Set the marker action.  Options are ADD and DELETE
		marker.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = 0.10;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0.10;
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(-1*delta_yaw);

		// Set the scale of the marker
		marker.scale.x = 0.4;
		marker.scale.y = 0.4;
		marker.scale.z = fabs(*linear_velocity)*0.5/max_linear_velocity;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 1.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		// Publish the marker
		marker_pub_ptr->publish(marker);
	}

	return delta_yaw;
}

void displayGoal(double goal_x, double goal_y)
{
	visualization_msgs::Marker goal_marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	goal_marker.header.frame_id = global_frame_id;
	goal_marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	goal_marker.ns = "goals";
	goal_marker.id = robots[me].id;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	goal_marker.type = visualization_msgs::Marker::CYLINDER;

	// Set the marker action.  Options are ADD and DELETE
	goal_marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	goal_marker.pose.position.x = goal_x;
	goal_marker.pose.position.y = goal_y;
	goal_marker.pose.position.z = 0.0;
	goal_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

	// Set the scale of the marker
	goal_marker.scale.x = goal_tolerance*2.0;
	goal_marker.scale.y = goal_tolerance*2.0;
	goal_marker.scale.z = 0.01;

	// Set the color -- be sure to set alpha to something non-zero!
	goal_marker.color.r = 0.0f;
	goal_marker.color.g = 1.0f;
	goal_marker.color.b = 0.0f;
	goal_marker.color.a = 0.5;

	goal_marker.lifetime = ros::Duration();

	// Publish the marker
	marker_pub_ptr->publish(goal_marker);
}

void publishVelocities(double linear_velocity, double angular_velocity)
{
	// Send the new velocities to the robot...
        geometry_msgs::Twist cmd_vel;
        
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = angular_velocity;
        cmd_vel_pub_ptr->publish(cmd_vel);
}

//  ********************* Simple goal *********************
void goalReceived(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ros::Time start_time = ros::Time::now();
    simple_goal++;

    if(state == SN_ROTATING_AS || state == SN_MOVING_AS)
    {
    	ROS_ERROR("MoveBase Simple - %s - Ignoring goal because there is already an ActionServer goal in progress!", __FUNCTION__);
    	return;
    }

    if(msg->header.frame_id != global_frame_id)
    {
    	ROS_ERROR("MoveBase Simple - %s - Ignoring goal because frame_id is not the global frame!", __FUNCTION__);
    	return;
    }

    geometry_msgs::PoseStamped goal = *msg;

    displayGoal(goal.pose.position.x, goal.pose.position.y);

    state = SN_MOVING;
    distance_travelled = 0.0;

    ros::Rate r(rate);
    while(n_ptr->ok())
    {
    	if(state == SN_ROTATING_AS || state == SN_MOVING_AS || simple_goal>1) return;

        double linear_velocity;
        double angular_velocity;
        
        double current_orientation = tf::getYaw(robots[me].pose.pose.orientation);
        double current_x = robots[me].pose.pose.position.x;
        double current_y = robots[me].pose.pose.position.y;

		// If we've been at this for too long...
		if(ros::Time::now() - start_time > goal_timeout)
		{
			publishVelocities(0.0, 0.0);
			state = SN_STOPPED;
			return;
		}
        
		// If we reached our target position
        if(sqrt((current_x-goal.pose.position.x)*(current_x-goal.pose.position.x)+(current_y-goal.pose.position.y)*(current_y-goal.pose.position.y)) < goal_tolerance)
        {
            publishVelocities(0.0, 0.0);
	    	state = SN_STOPPED;
	    	simple_goal--;
	    	return;
        }

		// If we travelled more than max distance pause
		if(distance_travelled > max_distance)
		{
			publishVelocities(0.0, 0.0);
			distance_travelled = 0.0;
			delay.sleep();
		}

		double delta_yaw = calculateVelocities(current_x, current_y, current_orientation, goal.pose.position.x, goal.pose.position.y, &linear_velocity, &angular_velocity);

		// If we are too far from our target orientation rotate in place
		if(state == SN_MOVING && fabs(delta_yaw) > angular_threshold)
		{
			state = SN_ROTATING;
			distance_travelled = 0.0;
			delay.sleep();
		}
	
		// If we intend to rotate before moving forward...
		if(state == SN_ROTATING)
		{
			linear_velocity = 0.0;
			angular_velocity = (angular_velocity < 0 ? -in_place_angular_velocity : in_place_angular_velocity);
			if(fabs(delta_yaw) < angular_threshold) angular_velocity = 0.0;
		}

		publishVelocities(linear_velocity, angular_velocity);
		if(state == SN_ROTATING && fabs(delta_yaw) < angular_threshold)
		{
			state = SN_MOVING;
			distance_travelled = 0.0;
			delay.sleep();
		}
        
        ros::spinOnce();
        r.sleep();
    }
}
//  *******************************************************

// ********************* Action server goal *********************
void asCallback(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
{
	ros::Time start_time = ros::Time::now();

	if(move_base_goal->target_pose.header.frame_id.compare(global_frame_id) != 0)
	{
		as->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because frame_id is not the global frame!");
		return;
	}

	geometry_msgs::PoseStamped goal = move_base_goal->target_pose;

	displayGoal(goal.pose.position.x, goal.pose.position.y);
    	
	state = SN_MOVING_AS;
	distance_travelled = 0.0;

	ros::Rate r(rate);
    	while(n_ptr->ok())
    	{
		if(as->isPreemptRequested())
		{
			publishVelocities(0.0, 0.0);
			as->setPreempted();
			state = SN_STOPPED;
			return;
		}

        	double linear_velocity;
        	double angular_velocity;
        
        	double current_orientation = tf::getYaw(robots[me].pose.pose.orientation);
        	double current_x = robots[me].pose.pose.position.x;
        	double current_y = robots[me].pose.pose.position.y;

		// If we've been at this for too long...
		if(ros::Time::now() - start_time > goal_timeout)
		{
			publishVelocities(0.0, 0.0);
	    	as->setAborted();
	    	state = SN_STOPPED;
			return;
		}
        
		// If we reached our target position 
        	if(sqrt((current_x-goal.pose.position.x)*(current_x-goal.pose.position.x)+(current_y-goal.pose.position.y)*(current_y-goal.pose.position.y)) < goal_tolerance)
        	{
            	publishVelocities(0.0, 0.0);
	    		as->setSucceeded();
	    		state = SN_STOPPED;
			return;
         	}

			// If we travelled more than max distance pause
			if(distance_travelled > max_distance)
			{
				publishVelocities(0.0, 0.0);
				distance_travelled = 0.0;
				delay.sleep();
			}
  
	    	double delta_yaw = calculateVelocities(current_x, current_y, current_orientation, goal.pose.position.x, goal.pose.position.y, &linear_velocity, &angular_velocity);

			// If we are too far from our target orientation rotate in place
			if(state == SN_MOVING_AS && fabs(delta_yaw) > angular_threshold)
			{
				state = SN_ROTATING_AS;
				distance_travelled = 0.0;
				delay.sleep();
			}

	    	// If we intend to rotate before moving forward...
	    	if(state == SN_ROTATING_AS)
	    	{
	    		linear_velocity = 0.0;
			angular_velocity = (angular_velocity < 0 ? -in_place_angular_velocity : in_place_angular_velocity);
	    		if(fabs(delta_yaw) < angular_threshold) angular_velocity = 0.0;
	    	}

		// Publish feedback
    		move_base_msgs::MoveBaseFeedback feedback;

    		feedback.base_position = robots[me].pose;
    		as->publishFeedback(feedback);

			publishVelocities(linear_velocity, angular_velocity);
			if(state == SN_ROTATING_AS && fabs(delta_yaw) < angular_threshold)
			{
				state = SN_MOVING_AS;
				distance_travelled = 0.0;
				delay.sleep();
			}
        
			ros::spinOnce();
			r.sleep();
	}
}
//  *************************************************************

void odomCallback(int index, const boost::shared_ptr<const nav_msgs::Odometry>& msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = msg->header.frame_id;
    pose.header.stamp = msg->header.stamp;
    pose.pose = msg->pose.pose;

    geometry_msgs::PoseStamped global_frame_id_pose;
            
    try 
    {
        // Transform the nose pose into a pose in the map frame
        tf_listener->transformPose(global_frame_id, pose, global_frame_id_pose);
    }
    catch(tf::TransformException &ex) 
    {
        ROS_ERROR("MoveBase Simple - %s - Error: %s", __FUNCTION__, ex.what());
        return;
    }

    if(index == me && max_distance > 0.0) distance_travelled += sqrt(pow(robots[index].pose.pose.position.x-global_frame_id_pose.pose.position.x, 2)+pow(robots[index].pose.pose.position.y-global_frame_id_pose.pose.position.y, 2));

    robots[index].pose = global_frame_id_pose;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	map = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base_simple");
    
    ROS_INFO("Move Base Simple for ROS");
    
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    n_ptr = &n;

    tf_listener = new tf::TransformListener();

    as = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(asCallback, _1), false);
    
    // Parameters
    pn.param<std::string>("global_frame_id", global_frame_id, "/map");

    // Lets load the list of robot ids...raries: libros.
    XmlRpc::XmlRpcValue list_of_ids;
    if( n.getParam("/list_of_ids", list_of_ids) )
    {
        int my_id;
        if( !pn.getParam("my_id", my_id) )
        {
            ROS_FATAL("MoveBase Simple -- A list of ids was defined but my_id was not!!!");
            ROS_BREAK();
        }
        
        ROS_ASSERT(list_of_ids.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
        for(int i=0 ; i<list_of_ids.size() ; ++i) 
        {
            ROS_ASSERT(list_of_ids[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
            
            robots.push_back(Robot(static_cast<int>(list_of_ids[i])));
            
            robots[i].odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>();
            std::string topic = robots[i].prefix;
            topic.append("/odom");
            robots[i].odom_sub->subscribe(n, topic, 20);
            robots[i].tf_filter = new tf::MessageFilter<nav_msgs::Odometry>(*robots[i].odom_sub, *tf_listener, global_frame_id, 20);
            robots[i].tf_filter->registerCallback( boost::bind(odomCallback, i, _1) );
            
            // Now you can access your data using robots[me].wahetever_you_need!
            if(robots[i].id == my_id) me = i;
        }
    }
    else
    // If a list of ids is not defined...
    {
    	ROS_FATAL("MoveBase Simple -- A list of ids was not defined!!!");
        ROS_BREAK();
    }
    
    pn.param("rate", rate, 3.0);
    pn.param("in_place_angular_velocity", in_place_angular_velocity, 3.0);
    pn.param("max_linear_velocity", max_linear_velocity, 0.2);
    pn.param("min_linear_velocity", min_linear_velocity, 0.05);
    pn.param("alpha", alpha, 0.5);
    pn.param("attraction_coefficient", attraction_coefficient, 0.5);
    pn.param("repulsion_coefficient_robots", repulsion_coefficient_robots, 0.5);
    pn.param("repulsion_coefficient_obstacles", repulsion_coefficient_obstacles, 0.5);
    pn.param("goal_tolerance", goal_tolerance, 0.10);
    pn.param("angular_threshold", angular_threshold, 0.4);
    pn.param("repulsion_threshold_robots", repulsion_threshold_robots, 0.5);
    pn.param("repulsion_threshold_obstacles", repulsion_threshold_obstacles, 0.0);
    double delay_sec = 0.0;
    pn.param("delay", delay_sec, 0.0);
    delay = ros::Duration(delay_sec);
    pn.param("visualization", visualization, false);
    pn.param("max_distance", max_distance, 0.5);
    double goal_timeout_sec = 0.0;
    pn.param("goal_timeout", goal_timeout_sec, 60.0);
    goal_timeout = ros::Duration(goal_timeout_sec);

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
    
    ROS_INFO("Using %s as the global frame.", global_frame_id.c_str());
	
    // Making all the publishing and subscriptions...
    std::string topic;

    topic = robots[me].prefix;
    topic.append("/cmd_vel");
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>(topic, 10);
    cmd_vel_pub_ptr = &cmd_vel_pub;

    ros::Publisher marker_pub;
    if(visualization)
    {
    	marker_pub = n.advertise<visualization_msgs::Marker>("/simple_navigation_markers", 10);
    	marker_pub_ptr = &marker_pub;
    }

    topic = robots[me].prefix;
    topic.append("/move_base_simple/goal");
    ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PoseStamped>(topic, 10, goalReceived);

    ros::Subscriber map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1, mapCallback);
    
    ROS_INFO("idx:%d id:%d %s", me, robots[me].id, topic.c_str());

    state = SN_STOPPED;

    // Start the action server!
    as->start();
    
    ros::spin();

    return(0);
}

// EOF

