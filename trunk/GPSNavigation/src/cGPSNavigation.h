#ifndef _CGPSNAVIGATION_H_
#define _CGPSNAVIGATION_H_

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.14159265


#define AUTODIFF 268.8
//#define AUTODIFF 266.5
#define AERODIFF 207.0

class cGPSNavigation
{

public:
    enum mode_t { MODE_AUTO, MODE_AERO};
private:
    double goalX;
    double goalY;

    double relativeX;
    double relativeY;

    double currentAngle;
    double angleToGoal;

    double GPSError;

    mode_t mode;

    double transformFromPose(geometry_msgs::PoseStampedPtr pose);
    double calculateAngleToGoal(geometry_msgs::PoseStampedPtr pose);

public:

    cGPSNavigation (double _goalX, double _goalY, double _GPSError = 1.5, mode_t _mode = MODE_AUTO);

    double getDirection(geometry_msgs::PoseStampedPtr pose);
    bool checkGoal(geometry_msgs::PoseStampedPtr pose);

    void setGoal(double x, double y)
    {
        this->goalX = x;
        this->goalY = y;
    }

    void setGoalX(double goalX)
    {
        this->goalX = goalX;
    }
    void setGoalY(double goalY)
    {
        this->goalY = goalY;
    }
    double getGoalX() const
    {
        return goalX;
    }
    double getGoalY() const
    {
        return goalY;
    }
    double getRelativeX() const
    {
        return relativeX;
    }
    double getRelativeY() const
    {
        return relativeY;
    }
    void setGPSError(double GPSError)
    {
        this->GPSError = GPSError;
    }
    double getGPSError() const
    {
        return GPSError;
    }
    double getAngleToGoal() const
    {
        return angleToGoal;
    }
    double getCurrentAngle() const
    {
        return currentAngle;
    }

};

#endif
