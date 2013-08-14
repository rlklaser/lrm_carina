#ifndef _CGPSNAVIGATION_H_
#define _CGPSNAVIGATION_H_

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose2D.h>

#define PI 3.14159265

class cGPSNavigation
{
	float goalX;
	float goalY;

	float relativeX;
	float relativeY;

	float currentAngle;
	float angleToGoal;

	float GPSError;

	float transformFromPose(geometry_msgs::Pose2DPtr pose);
	float calculateAngleToGoal(geometry_msgs::Pose2DPtr pose);

public:

	cGPSNavigation (float _goalX, float _goalY, float _GPSError = 1.5f);


	float getDirection(geometry_msgs::Pose2DPtr pose);
	bool checkGoal(geometry_msgs::Pose2DPtr pose);

	void setGoalX(float goalX)
	{
		this->goalX = goalX;
	}
	void setGoalY(float goalY)
	{
		this->goalY = goalY;
	}
	float getGoalX() const
	{
		return goalX;
	}
	float getGoalY() const
	{
		return goalY;
	}
	float getRelativeX() const
	{
		return relativeX;
	}
	float getRelativeY() const
	{
		return relativeY;
	}
	void setGPSError(float GPSError)
	{
		this->GPSError = GPSError;
	}
	float getGPSError() const
	{
		return GPSError;
	}
	float getAngleToGoal() const
	{
		return angleToGoal;
	}
	float getCurrentAngle() const
	{
		return currentAngle;
	}

};

#endif
