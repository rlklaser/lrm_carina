#include "cVFH.h"

cVFH::~cVFH()
{
}

cVFH::cVFH(bool _bGenerateImage, int _threshold, float _maxAcel, float _minAcel, int _sMax, int _sMin):
bGenerateImage(_bGenerateImage), threshold(_threshold), maxAcel(_maxAcel), minAcel(_minAcel), sMax(_sMax), sMin(_sMin)
{
	state = TOGOAL;
	imageH = 100;
}

float cVFH::calcule(const vector<int> &_sectorHeight, float _angle)
{
	
	float sMaxR = sMax;
	float sMinR = sMin ;
	
	
	vector<pair<int, int> > vales;
	
	if(bGenerateImage)
		generateImage(_sectorHeight);
		
	bool onLimiar = false;

	for(unsigned int i=0; i< _sectorHeight.size(); i++)  //Detect vales
	{
		if(onLimiar)
		{
			if(_sectorHeight[i] > threshold)
			{
				vales.back().second = i;
				onLimiar = false;
			}
		}
		else if(_sectorHeight[i] < threshold)
		{
			vales.push_back(make_pair(i, 0));
			onLimiar = true;
		}
	}

	if(onLimiar)
	{
		vales.back().second = _sectorHeight.size() - 1;
	}

	for(unsigned int i=0; i<vales.size(); )          // Remove small vales based on sMin
	{
		cout<<"Vale N"<<i<<" s:"<< vales[i].first <<" e:"<< vales[i].second << endl;
		if((vales[i].second - vales[i].first) < sMinR)
		{
			vales.erase(vales.begin() + i);
		}
		else  i++;
	}
	
	int distVale = -1;
	int dist = _angle; //scalar
	dist += _sectorHeight.size() / 2;  
	if(dist > (int)_sectorHeight.size())
		dist = _sectorHeight.size();
	else if(dist < 1)
		dist = 1;

	for(unsigned int i=0; i<vales.size(); i++)    //Select vale
	{
		if(dist < vales[i].first)
		{
			if(i==0)
				distVale = i;
			else if((vales[i].first - dist)  > (dist -  vales[i-1].second))
				distVale = i-1;
			else
				distVale = i;
			break;
		}
		else if(dist >= vales[i].first && dist <= vales[i].second)
		{
			distVale = i;
			break;
		}
	}

	int angleS = 0;
	if(vales.empty())
		angleS=_sectorHeight.size()/2;
	else if(distVale == -1)
		distVale = vales.size()-1;
	//cout<<"Dist vale:" << distVale <<endl;

	if(distVale != -1)    //     Select Angle
	{
		int valeSize = (vales[distVale].second - vales[distVale].first);


		if(valeSize < sMaxR*2)
			angleS = (vales[distVale].second + vales[distVale].first) / 2;
		else if((abs(vales[distVale].first - dist))  > (abs(dist -  vales[distVale].second)))
		{
			if((vales[distVale].second - dist) > sMaxR)
				angleS = dist;
			else
				angleS = vales[distVale].second - sMaxR;
		}
		else
		{
			if((dist - vales[distVale].first) > sMaxR)
				angleS = dist;
			else
				angleS = vales[distVale].first + sMaxR;
		}
		
		if(state == CHANGE)
		{
			state = TOGOAL;
		}
		
		if(state == ONOBSTACLE)
		{
			if(angleS == dist || (vales[distVale].first <= 0 && vales[distVale].second == _sectorHeight.size()-1) )
			{
				state = CHANGE;
			}
		}
		
		if(state == TOGOAL)
		{
			if(vales[distVale].first > 0 || vales[distVale].second != _sectorHeight.size()-1)
				state = ONOBSTACLE;
		}


	}
	
	
	if(bGenerateImage)
	{
		cv::Point pT(0, imageH - threshold);
		cv::Point pT2(_sectorHeight.size(), imageH - threshold);
		cv::line(img, pT, pT2, CV_RGB(255, 0, 0), 2);
		pT.x = _sectorHeight.size()/2;
		pT.y = imageH;
		pT2.x = angleS;
		if(distVale == -1)
			pT2.y = imageH;
		else pT2.y = imageH-80;
		cv::line(img, pT, pT2, CV_RGB(0, 200, 0), 3);
	}
	


	float scaledAngle = angleS - ((int)_sectorHeight.size() / 2);
	
	if(distVale == -1)
		acel = 0;
	else acel = maxAcel * (1 / ( abs( scaledAngle ) + 1)) + minAcel;
	
	cout<<"Dist: "<<dist<<" AngleS: " << angleS <<" Scaled Angle: " << scaledAngle << " Acel: " << acel << endl;
	
	return scaledAngle;
}

float cVFH::getAcel()
{
	return acel;
}

cv::Mat cVFH::getImage()
{
	return img;
}

void cVFH::generateImage(const vector<int> &_sectorHeight)
{
	img.create(cv::Size(_sectorHeight.size(), imageH) , CV_8UC3);
	img = cv::Scalar(0);
	cv::Point pT, pT2;
	pT.y = imageH - _sectorHeight[0];
	pT.x = 0;
	
	for(unsigned int i=1; i< _sectorHeight.size(); i++)
	{
		pT2.y = imageH - _sectorHeight[i];
		pT2.x = i;
		cv::line(img, pT, pT2, CV_RGB(100, 0, 150), 2);
		pT = pT2;
	}
	
}

