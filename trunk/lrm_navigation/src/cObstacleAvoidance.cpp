#include "cObstacleAvoidance.h"

cObstacleAvoidance::cObstacleAvoidance() {
	newImage = newPointCloud = running = false;
	angleScalar = 0.0f;

	cv::namedWindow("VFH", CV_WINDOW_NORMAL);
	cv::namedWindow("Obstacles", CV_WINDOW_NORMAL);

	state = TOGOAL;

	//vfh = new cVFH(false, 68, 0.35, 0.2, 25, 25);
	vfh = new cVFH(true, 68, 1.0, 0.2, 25, 25);

	obsDetection = new cObstacleDetection(3, 3, 0.5, 0.2);
	findPlane = new cFindPlane(0.004, 1000, Eigen::Vector3f(0, -0.5, -0.5), 0.6);
	gpsNavigation = new cGPSNavigation(0, 0);
	//robot = new PlayerClient("localhost");
	//carina = new Position2dProxy(robot, 0);

	//carina->SetMotorEnable(true);
}

cObstacleAvoidance::~cObstacleAvoidance() {
	delete vfh;
}

void cObstacleAvoidance::pointCloudCallback(PointCloud::Ptr _msg) {
	staticPointCloud = _msg->makeShared();

	newPointCloud = true;

	if (newPointCloud && newImage)
		process();

}

void cObstacleAvoidance::setGoal(float _goalX, float _goalY) {
	gpsNavigation->setGoalX(_goalX);
	gpsNavigation->setGoalY(_goalY);
}

void cObstacleAvoidance::poseCallback(geometry_msgs::Pose2DPtr _pose) {
	staticPose = _pose;
}

void cObstacleAvoidance::imageCallback(const sensor_msgs::ImageConstPtr& img) {
	//cv::Mat tempimg = bridge_.imgMsgToCv(img, "bgr8");
	//staticImage = tempimg.clone();

	//cv_bridge::CvImagePtr cvimg = cv_bridge::toCvCopy(img, "rgb8");
	cv_bridge::CvImageConstPtr cvimg = cv_bridge::toCvShare(img, "rgb8");

	staticImage = cvimg->image.clone();

	newImage = true;

	if (newPointCloud && newImage)
		process();
}

void cObstacleAvoidance::process() {
	newImage = newPointCloud = false;
	static float obsAngle, obsAcel, obsPosX, obsPosY;
	static vector<float> angles, acels;
	cv::Mat& obsImg = staticImage;

	if (angleScalar == 0.0f)
		findPlane->calculatePlane(staticPointCloud, translateY, translateZ, angleScalar);

	findPlane->rotatePlaneToZ(staticPointCloud, translateY, translateZ, angleScalar);

	pubPointCloud->publish(staticPointCloud);
//	std::cout << "published" << std::endl;

	obsDetection->detect(staticPointCloud, obsImg);
	//std::cout << "detected" << std::endl;

	obsDetection->generateImage(obsImg);
	//std::cout << "image generated" << std::endl;

	vector<int> sectorHeight;

	sectorHeight.resize(90, 0);

	//std::cout << "looping" << std::endl;

	for (unsigned int iM = 0; iM < staticPointCloud->height; iM++) {

		for (unsigned int jM = 0; jM < staticPointCloud->width; jM++) {
			cv::Vec3b valPixel = obsImg.at<cv::Vec3b>(iM, jM);

			float x = staticPointCloud->points[iM * staticPointCloud->width + jM].x;
			float y = staticPointCloud->points[iM * staticPointCloud->width + jM].y;
			float z = staticPointCloud->points[iM * staticPointCloud->width + jM].z;

			if (valPixel[0] == 0 && valPixel[1] == 0) {
				int angle = (int) (atan2(z, -x) * 180.0 / 3.14);
				angle = -angle + 135;
				if (angle > 89)
					angle = 89;
				else if (angle < 0)
					angle = 0;
				sectorHeight[angle] += 5 / sqrt((pow(-x, 2) + pow(y, 2) + pow(z, 2)));
			}

		}

	}

	if (staticPose == NULL)
		return;

	float direction = gpsNavigation->getDirection(staticPose);

	printf("GoalX: %f GoalY: %f \nPoseX: %f Pose Y: %f \nRelX: %f RelY: %f\n", gpsNavigation->getGoalX(), gpsNavigation->getGoalY(), staticPose->x, staticPose->y, gpsNavigation->getRelativeX(), gpsNavigation->getRelativeY());
	printf("Angle: %f  Dist angle: %f Relative angle: %f\n", gpsNavigation->getCurrentAngle(), gpsNavigation->getAngleToGoal(), direction);
	printf("State: %d\n", state);
	printf("Obs Angle: %f\n", obsAngle);

	if (gpsNavigation->checkGoal(staticPose)) {
		cout << "THE END!! :)" << endl;
		running = false;
		//carina->SetSpeed(0.0, 0.0);
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0;
		pubCommand->publish(cmd_vel);

	}

	float vfhAngle = vfh->calcule(sectorHeight, direction);
	cv::Mat vfhImg = vfh->getImage();
	float acel = vfh->getAcel();

	printf("VFH State: %d\n", vfh->getState());

	//POR ODOMETRIA

	/*if(state == TOGOAL)
	 {
	 if(vfh->getState() == CHANGE)
	 {
	 carina->ResetOdometry();
	 obsAngle = prevAngle;
	 obsAcel = prevAcel;
	 state = ONOBSTACLE;
	 }
	 }
	 else if (state == ONOBSTACLE)
	 {
	 if (sqrt(carina->GetXPos()*carina->GetXPos() + carina->GetYPos()*carina->GetYPos()) > 2)
	 {
	 state = TOGOAL;
	 }
	 }*/

	// POR GPS
	if (state == TOGOAL) {
		if (vfh->getState() == CHANGE) {
			unsigned int size = acels.size();
			float prevAngle = angles[size - 1];
			float diffAngle = vfhAngle - prevAngle;

			int i = size - 2;

			if (diffAngle > 0) {
				while (angles[i] <= angles[i + 1]) {
					i--;
				}
			} else {
				while (angles[i] >= angles[i + 1]) {
					i--;
				}
			}

			obsPosX = staticPose->x;
			obsPosY = staticPose->y;
			obsAngle = 0.0;
			obsAcel = acels[i + 1];
			state = ONOBSTACLE;
		}
	} else if (state == ONOBSTACLE) {
		if (sqrt((staticPose->x - obsPosX) * (staticPose->x - obsPosX) + (staticPose->y - obsPosY) * (staticPose->y - obsPosY)) > 1.8 || vfh->getState() == ONOBSTACLE) {
			state = TOGOAL;
		}
	}

	/*CvFont font = cv::fontQt("Times");
	 cv::addText( obsImg, CarStateStr[state], cv::Point(15,50), font);
	 cv::addText( vfhImg, CarStateStr[vfh->getState()], cv::Point(15,50), font);*/
	cv::imshow("VFH", vfhImg);
	cv::imshow("Obstacles", obsImg);

	char key = cv::waitKey(20);

	if (key == 'r') {
		angleScalar = 0.0f;
	}

	if (key == 's')
		running = true;

	if (key == 'd') {
		running = false;
		//carina->SetSpeed(0.0, 0.0);
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0;
		pubCommand->publish(cmd_vel);

	}

	if (key == 'g')
		state = TOGOAL;

	if (running) {
		printf("running...\n");
		if (state == TOGOAL) {
			//carina->SetSpeed(acel, (vfhAngle *-1) /25);
			cmd_vel.linear.x = acel;
			cmd_vel.angular.z = (vfhAngle * -1) / 25;
		} else {
			//carina->SetSpeed(obsAcel, (obsAngle *-1) /25);
			cmd_vel.linear.x = obsAcel;
			cmd_vel.angular.z = (obsAngle * -1) / 25;
		}
		pubCommand->publish(cmd_vel);
	}

	printf("To carina: %f\n\n\n", (vfhAngle * -1) / 25);

	acels.push_back(acel);
	angles.push_back(vfhAngle);

}
