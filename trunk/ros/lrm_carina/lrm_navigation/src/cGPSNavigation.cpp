#include "cGPSNavigation.h"


cGPSNavigation::cGPSNavigation(float _goalX, float _goalY, float _GPSError):
goalX(_goalX), goalY(_goalY), GPSError(_GPSError)
{
}

bool cGPSNavigation::checkGoal(geometry_msgs::Pose2DPtr pose)
{
	if(fabs(goalX - pose->x) < GPSError &&  fabs(goalY - pose->y) < GPSError)
		return true;
	else return false;
}

float cGPSNavigation::calculateAngleToGoal(geometry_msgs::Pose2DPtr pose)
{
	relativeX = goalX - pose->x;
	relativeY = goalY - pose->y;
	
	float distAngle = atan2(relativeX, relativeY) * 180 /PI;

	distAngle = 90 - distAngle;

	if(distAngle < 0)
		distAngle = 360 + distAngle;
		
	return distAngle;
}

float cGPSNavigation::getDirection(geometry_msgs::Pose2DPtr pose)
{
	currentAngle = this->transformFromPose(pose);
	
	angleToGoal = this->calculateAngleToGoal(pose);
	
	float angle = currentAngle - angleToGoal;

	if(angle< -180)
		angle = 360 + angle;

	if(angle > 180)
	{
		angle = -360 + angle;
	}
	
	return angle;
}

float cGPSNavigation::transformFromPose(geometry_msgs::Pose2DPtr pose)
{
	currentAngle = pose->theta;
	
	currentAngle -= 250;

	if(currentAngle < 0)
		currentAngle = 360 + currentAngle;
		
	return currentAngle;
}

