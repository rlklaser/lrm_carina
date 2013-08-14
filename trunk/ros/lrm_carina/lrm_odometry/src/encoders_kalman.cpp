/*
 *  Copyright (C) 2012, Laboratorio de Robotica Movel - ICMC/USP
 *  Rafael Luiz Klaser <rlklaser@gmail.com>
 *  http://lrm.icmc.usp.br
 *
 *  Apoio FAPESP: 2012/04555-4
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file encoders_kalman.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Oct 16, 2012
 *
 */

#include <ros/ros.h>
#include <atuacao/Encoders.h>
#include <opencv2/opencv.hpp>

ros::Publisher encoder_publisher;

CvKalman* kalman;
CvMat* x_k;
// Process noise
CvMat* w_k;
// Measurements, only one parameter for angle
CvMat* z_k;

bool filter_wheel;

void publishEncoder(long wheel, long steer)
{
	atuacao::Encoders encoder;

	encoder.header.frame_id = "/wheel_encoder_filtered";
	encoder.header.stamp = ros::Time::now();

	encoder.relative.push_back(wheel);
	encoder.absolute.push_back(steer);

	encoder_publisher.publish(encoder);
}

void encodersCallback(const atuacao::Encoders::ConstPtr& encoder)
{
	long encWheelValue = encoder->relative.front();
	long encSteerValue = encoder->absolute.front();

	const CvMat* y_k = cvKalmanPredict(kalman, 0);

	cvmSet(z_k, 0, 0, encSteerValue);

	cvMatMulAdd(kalman->measurement_matrix, x_k, z_k, z_k);

	double estimated = y_k->data.fl[0];

	// Adjust Kalman filter state
	cvKalmanCorrect(kalman, z_k);
	cvMatMulAdd(kalman->transition_matrix, x_k, w_k, x_k);

	publishEncoder(encWheelValue, estimated);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "encoders_kalman");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	//KalmanFilter::KalmanFilter

	kalman = cvCreateKalman(1, 1, 0);
	x_k = cvCreateMat(1, 1, CV_32FC1);
	// Process noise
	w_k = cvCreateMat(1, 1, CV_32FC1);
	// Measurements, only one parameter for angle
	z_k = cvCreateMat(1, 1, CV_32FC1);

	cvZero(z_k);

	// Transition matrix F describes model parameters at and k and k+1
	const float F[] = {1, 1, 0, 1};
	memcpy(kalman->transition_matrix->data.fl, F, sizeof(F));

	// Initialize other Kalman parameters
	cvSetIdentity(kalman->measurement_matrix, cvRealScalar(1));
	cvSetIdentity(kalman->process_noise_cov, cvRealScalar(1e-5));
	cvSetIdentity(kalman->measurement_noise_cov, cvRealScalar(1e-1));
	cvSetIdentity(kalman->error_cov_post, cvRealScalar(1));

	ros::Subscriber encoders_sub = nh.subscribe<atuacao::Encoders>("encoders", 1, &encodersCallback);
	encoder_publisher = nh.advertise<atuacao::Encoders>("encoders_filtered", 1, 0);

	ros::spin();

	cvReleaseKalman(&kalman);
	//cvReleaseMat(&x_k);
	//cvReleaseMat(&w_k);
	//cvReleaseMat(&z_k);

	return 0;
}
