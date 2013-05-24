#include "cGPSNavigation.h"


cGPSNavigation::cGPSNavigation(double _goalX, double _goalY, double _GPSError, mode_t _mode):
    goalX(_goalX), goalY(_goalY), GPSError(_GPSError), mode(_mode)
{
}

bool cGPSNavigation::checkGoal(geometry_msgs::PoseStampedPtr pose)
{
	if( sqrt(pow(goalX - pose->pose.position.x, 2) + pow(goalY - pose->pose.position.y, 2)) < GPSError)
		return true;

    return false;
}

double cGPSNavigation::calculateAngleToGoal(geometry_msgs::PoseStampedPtr pose)
{
	relativeX = goalX - pose->pose.position.x;
	relativeY = goalY - pose->pose.position.y;
	
	double distAngle = atan2(relativeX, relativeY) * 180 /PI;

	distAngle = 90 - distAngle;

	if(distAngle < 0)
		distAngle = 360 + distAngle;
		
	return distAngle;
}

double cGPSNavigation::getDirection(geometry_msgs::PoseStampedPtr pose)
{
	currentAngle = this->transformFromPose(pose);
	
	angleToGoal = this->calculateAngleToGoal(pose);
	
	double angle = currentAngle - angleToGoal;

        if(angle < -180)
		angle = 360 + angle;
        else if(angle > 180)
		angle = -360 + angle;
	
	return angle;
}

double cGPSNavigation::transformFromPose(geometry_msgs::PoseStampedPtr pose)
{
	currentAngle = pose->pose.orientation.z;
	
        if(mode == MODE_AUTO)
            currentAngle -= AUTODIFF;
        else currentAngle -= AERODIFF;

	if(currentAngle < 0)
		currentAngle = 360 + currentAngle;
		
	return currentAngle;
}

