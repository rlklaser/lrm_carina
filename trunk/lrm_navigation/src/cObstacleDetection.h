#ifndef _COBSTACLEDETECTION_H_
#define _COBSTACLEDETECTION_H_

//#define OPENCL
#pragma clang diagnostic ignored "-Winvalid-offsetof"

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <iterator>
#include <fstream>
#include <iostream>
#ifdef OPENCL
#include <CL/cl.hpp>
#endif

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


#include <iostream>

using namespace std;

class cObstacleDetection
{
	int totalSizeV, totalSizeH;
	int localSizeV, localSizeH;
	int beginV, beginH;
	
	float threshold;
	float confiance;

	float * data;
	float * fImg;

#ifdef OPENCL
	cl::Context * context;
	cl::Buffer * dataBuffer;
	cl::Buffer * imgBuffer;
	vector<cl::Device> devices;
	cl::Kernel * kernel;
	cl::CommandQueue * queue;
#endif

public:

	cObstacleDetection(int _sizeX, int _sizeY, float _threshold, float _confiance);
	cObstacleDetection() {}
	
	void detect(PointCloud::Ptr pointcloud, cv::Mat& img);
	void generateImage(cv::Mat& img);
	~cObstacleDetection();
};

#endif
