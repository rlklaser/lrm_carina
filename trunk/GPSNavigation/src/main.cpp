#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <vector>
#include <utility>
#include <atuacao/SteeringAngle.h>
#include <atuacao/Throttle.h>
#include <atuacao/Brake.h>

bool sterringActive = false;
double sterringFactor = 0.0;
double maxSterring = 0.0;
double minDist = 0.0;
std::string pointsDir;
std::string pointsFileName;

using namespace std;

#include "cGPSNavigation.h"

typedef struct
{
    double x;
    double y;
    int v;
} PointV;

cGPSNavigation * naviCurrent;
cGPSNavigation * naviProx;
vector<PointV> gpsPoints;
int numberOfPoints;
int currentPoint;
int proxPoint;
atuacao::SteeringAngle steer;

ros::Publisher steering_pub;

void callback(const geometry_msgs::PoseStampedPtr pose)
{
    double relativeAngleCurrent = naviCurrent->getDirection(pose);
    double relativeAngleProx = naviProx->getDirection(pose);


    printf("GoalX: %f GoalY: %f \nPoseX: %f Pose Y: %f \nRelX: %f RelY: %f\n", naviCurrent->getGoalX(), naviCurrent->getGoalY(), pose->pose.position.x, pose->pose.position.y, naviCurrent->getRelativeX(), naviCurrent->getRelativeY());
    printf("Angle: %f  Dist angle Cur: %f Relative angle cur: %f Dist angle Prox: %f Relative angle prox: %f\n", naviCurrent->getCurrentAngle(), naviCurrent->getAngleToGoal(), relativeAngleCurrent, naviProx->getAngleToGoal(), relativeAngleProx);

    if(naviCurrent->checkGoal(pose))
    {
        cout << "Going to next Point!!!" << endl;
        cout << "Going to next Point!!!" << endl;
        cout << "Going to next Point!!!" << endl;
        currentPoint ++;
        proxPoint ++;
        if(proxPoint == numberOfPoints)
            proxPoint = 0;
        if(currentPoint == numberOfPoints)
            proxPoint = 0;
        naviCurrent->setGoalX(gpsPoints[currentPoint].x);
        naviCurrent->setGoalY(gpsPoints[currentPoint].y);
        naviProx->setGoalX(gpsPoints[proxPoint].x);
        naviProx->setGoalY(gpsPoints[proxPoint].y);

        return;
    }


    double relativeAngle;

    double dist = sqrt (naviCurrent->getRelativeX()*naviCurrent->getRelativeX() + naviCurrent->getRelativeY()*naviCurrent->getRelativeY());

    if(dist < minDist)
    {
        printf("min dist\n");
        relativeAngle = (relativeAngleCurrent + relativeAngleProx) / 2;
    }
    else relativeAngle = relativeAngleCurrent;

    double stering = relativeAngle * sterringFactor;

    if(stering > maxSterring)
        stering = maxSterring;
    if(stering < -maxSterring)
        stering = -maxSterring;


    steer.angle = stering * -1.0;
    if(sterringActive)
        steering_pub.publish(steer);

    cout << "Going to point: " << currentPoint + 1 << endl << endl << endl;

}

void readList(vector<PointV> & list, const char * fileName)
{
    FILE * fHandle;

    fHandle = fopen(fileName, "rt");

    while(!feof(fHandle))
    {
        PointV p;
        fscanf(fHandle, "%lf %lf %d\n", &p.x, &p.y, &p.v);
        list.push_back(p);
    }

    fclose(fHandle);
}

void displayList(vector<PointV> & list)
{
    unsigned int i;

    for(i = 0; i < list.size(); i++)
    {
        printf("Point %d: %.2f, %.2f  Vel: %d\n", i+1, list[i].x, list[i].y, list[i].v);
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "GPSNavigation");
    ros::NodeHandle nh;

    nh.param("/GPSNavigation/ster", sterringActive, false);
    nh.param("/GPSNavigation/minDist", minDist, 3.0);
    nh.param("/GPSNavigation/sterringFactor", sterringFactor, 0.5);
    nh.param("/GPSNavigation/maxSterring", maxSterring, 25.0);
    nh.param("/GPSNavigation/pointsFileName", pointsFileName,  std::string("points.txt"));
    nh.param("/GPSNavigation/pointsDir", pointsDir, std::string(""));

    pointsDir += pointsFileName;

    readList(gpsPoints, pointsDir.c_str());
    numberOfPoints = (int) gpsPoints.size();
    displayList(gpsPoints);
    currentPoint = 0;
    proxPoint = 1;

    steer.angle = 0;

    naviCurrent = new cGPSNavigation(gpsPoints[currentPoint].x, gpsPoints[currentPoint].y, 2.0f, cGPSNavigation::MODE_AUTO);
    naviProx = new cGPSNavigation(gpsPoints[proxPoint].x, gpsPoints[proxPoint].y, 2.0f, cGPSNavigation::MODE_AUTO);
    ros::Subscriber subPose = nh.subscribe("/lse_xsens_mti/xsens_1/pose", 1, callback);

    if(sterringActive)
	steering_pub = nh.advertise<atuacao::SteeringAngle> ("steering_commands",1);

    ros::spin();

    return 0;
}
