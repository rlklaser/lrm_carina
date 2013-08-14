#ifndef _CFINDPLANE_H_
#define _CFINDPLANE_H_

#pragma clang diagnostic ignored "-Winvalid-offsetof"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/ModelCoefficients.h"

#include <Eigen/Core>

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include <pcl/sample_consensus/sac_model_plane.h>
#include "pcl/segmentation/sac_segmentation.h"

class cFindPlane
{
	float threshold;
	int maxIterations;
	Eigen::Vector3f axis;
	float epsThreshold;
	
	pcl::ModelCoefficients coefficients;
	pcl::PointIndices inliers;

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud);
	
public:
	
	cFindPlane(float _threshold, int _maxIterations, Eigen::Vector3f _axis, float _epsThreshold);
	void calculatePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, float &translatey, float &translatez, float &angleScalar);
	void rotatePlaneToZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, float translatey, float translatez, float angleScalar);
	
};

#endif
