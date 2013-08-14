
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#pragma clang diagnostic ignored "-Winvalid-offsetof"
#pragma clang diagnostic ignored "-Wextra-tokens"

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>


#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <Eigen/Core>

#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

#include <sstream>

#include "cObstacleAvoidance.h"

int main( int argc, char **argv)
{
	cObstacleAvoidance obstacleAvoidance;

	//obstacleAvoidance.setGoal(200793, 7563878);
	obstacleAvoidance.setGoal(10, 0);

	ros::init(argc, argv, "obstacleDetect");
	ros::NodeHandle n;

	image_transport::ImageTransport it(n);
	ros::Subscriber sub = n.subscribe("StereoMatcher/pointCloud", 1, &cObstacleAvoidance::pointCloudCallback, &obstacleAvoidance);
	ros::Subscriber subPose = n.subscribe("IMU/relativePose", 1, &cObstacleAvoidance::poseCallback, &obstacleAvoidance);
	image_transport::Subscriber subImg = it.subscribe("stereoCam/imageLeft", 1, &cObstacleAvoidance::imageCallback, &obstacleAvoidance);

	//plane_pub = n.advertise<geometry_msgs::PolygonStamped>("plane", 1);
	//pointPub = n.advertise<PointCloud>("onZPointCloud", 1);
	ros::Publisher pubPointCloud = n.advertise<PointCloud >("onZPointCloud", 1);
	ros::Publisher pubCommand = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	obstacleAvoidance.setPublisher(&pubPointCloud);
	obstacleAvoidance.setCommandPublisher(&pubCommand);

	ros::spin();

	return 0;
}
