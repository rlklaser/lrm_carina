#ifndef _CVFH_H_
#define _CVFH_H_

#pragma clang diagnostic ignored "-Winvalid-offsetof"

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

using namespace std;

enum CarState { ONOBSTACLE, TOGOAL, CHANGE };

class cVFH
{
       //Parameters
	bool bGenerateImage;
	int threshold;
	float maxAcel, minAcel;
	float sMax, sMin;

	CarState state;

	vector<int> sectorHeight;

	float goalX, goalY;

	cv::Mat img;
	int imageH;
	float acel;

	void generateImage(const vector<int> &_sectorHeight);


public:

	cVFH(bool _bGenerateImage, int _threshold, float _maxAcel, float _minAcel, int _sMax, int _sMin);
	float calcule(const vector<int> &_sectorHeight, float _angle);
	float getAcel();
	cv::Mat getImage();
	~cVFH();


	void setGenerateImage(bool bGenerateImage)
	{
		this->bGenerateImage = bGenerateImage;
	}
	void setMaxAcel(float maxAcel)
	{
		this->maxAcel = maxAcel;
	}
	void setMinAcel(float minAcel)
	{
		this->minAcel = minAcel;
	}
	void setSMax(int sMax)
	{
		this->sMax = sMax;
	}
	void setSMin(int sMin)
	{
		this->sMin = sMin;
	}
	void setThreshold(int threshold)
	{
		this->threshold = threshold;
	}
	bool getGenerateImage() const
	{
		return bGenerateImage;
	}
	float getMaxAcel() const
	{
		return maxAcel;
	}
	float getMinAcel() const
	{
		return minAcel;
	}
	int getSMax() const
	{
		return sMax;
	}
	int getSMin() const
	{
		return sMin;
	}
	int getThreshold() const
	{
		return threshold;
	}
	const CarState& getState() const
	{
		return state;
	}
};

#endif
