#ifndef _COBSTACLEAVOIDANCE_H_
#define _COBSTACLEAVOIDANCE_H_

#pragma clang diagnostic ignored "-Winvalid-offsetof"
#pragma clang diagnostic ignored "-Woverloaded-virtual"

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <cmath>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>

#include <vector>

//#include <libplayerc++/playerc++.h>

#include "cVFH.h"
#include "cObstacleDetection.h"
#include "cFindPlane.h"
#include "cGPSNavigation.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

//using namespace PlayerCc;

class cObstacleAvoidance
{
	bool running;
	bool newImage, newPointCloud;
	CarState state;

	cVFH * vfh;
	cObstacleDetection * obsDetection;
	cFindPlane * findPlane;
	cGPSNavigation * gpsNavigation;
	
	PointCloud::Ptr staticPointCloud;
	cv::Mat staticImage;
	//sensor_msgs::CvBridge bridge_;
	//geometry_msgs::PoseStampedPtr staticPose;
	geometry_msgs::Pose2DPtr staticPose;
	
	
	float translateY;
	float translateZ;
	float angleScalar;
	
	//PlayerClient * robot;
	//Position2dProxy * carina;

	ros::Publisher * pubPointCloud;
	ros::Publisher * pubCommand;
	
	geometry_msgs::Twist cmd_vel;

public:
	cObstacleAvoidance();
	void process();
	void pointCloudCallback(PointCloud::Ptr _msg);
	void poseCallback(geometry_msgs::Pose2DPtr _pose);
	void imageCallback(const sensor_msgs::ImageConstPtr& img);
	void setGoal(float _goalX, float _goalY);
	void setPublisher(ros::Publisher * pubPointCloud) { this->pubPointCloud = pubPointCloud; }
	void setCommandPublisher(ros::Publisher * pubCommand) { this->pubCommand = pubCommand; }
	~cObstacleAvoidance();
};

#endif
