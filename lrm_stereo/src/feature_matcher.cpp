/*
 *  Copyright (C) 2012-2013, Laboratorio de Robotica Movel - ICMC/USP
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
 * @file feature_matcher.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 20, 2013
 *
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/stereo_camera_model.h>
//#include <opencv2/calib3d/calib3d.hpp>

//#include <sensor_msgs/image_encodings.h>
//#include <stereo_msgs/DisparityImage.h>

//#include <lrm_stereo/DisparitySGBMConfig.h>
//#include <dynamic_reconfigure/server.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace lrm_stereo {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace sensor_msgs;
using namespace cv;
using namespace cv_bridge;
using namespace message_filters::sync_policies;

class FeatureMatcherNodelet: public nodelet::Nodelet {
	boost::shared_ptr<image_transport::ImageTransport> it_;

	// Subscriptions
	image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
	message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
	typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
	typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
	typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
	typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
	boost::shared_ptr<ExactSync> exact_sync_;
	boost::shared_ptr<ApproximateSync> approximate_sync_;
	// Publications
	boost::mutex connect_mutex_;

	ros::Publisher pub_matches_;

	image_transport::Publisher pub_image_;

	// Dynamic reconfigure
//	boost::recursive_mutex config_mutex_;
//	typedef lrm_stereo::DisparitySGBMConfig Config;
//	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
//	boost::shared_ptr<ReconfigureServer> reconfigure_server_;

// Processing state (note: only safe because we're single-threaded!)
	image_geometry::StereoCameraModel model_;

	virtual void onInit();

	void connectCb();

	void imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg, const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg);

	PointCloud cloud_msg;
	ros::Publisher cloud_pub;
};

void FeatureMatcherNodelet::onInit() {
	ros::NodeHandle &nh = getNodeHandle();
	ros::NodeHandle &private_nh = getPrivateNodeHandle();

	it_.reset(new image_transport::ImageTransport(nh));

	// Synchronize inputs. Topic subscriptions happen on demand in the connection
	// callback. Optionally do approximate synchronization.
	int queue_size;
	private_nh.param("queue_size", queue_size, 5);
	bool approx;
	private_nh.param("approximate_sync", approx, false);
	if (approx) {
		approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size), sub_l_image_, sub_l_info_, sub_r_image_, sub_r_info_));
		approximate_sync_->registerCallback(boost::bind(&FeatureMatcherNodelet::imageCb, this, _1, _2, _3, _4));
	} else {
		exact_sync_.reset(new ExactSync(ExactPolicy(queue_size), sub_l_image_, sub_l_info_, sub_r_image_, sub_r_info_));
		exact_sync_->registerCallback(boost::bind(&FeatureMatcherNodelet::imageCb, this, _1, _2, _3, _4));
	}

//	// Set up dynamic reconfiguration
//	ReconfigureServer::CallbackType f = boost::bind(&DisparitySGBMNodelet::configCb, this, _1, _2);
//	reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
//	reconfigure_server_->setCallback(f);

	// Monitor whether anyone is subscribed to the output
	//ros::SubscriberStatusCallback connect_cb = boost::bind(&FeatureMatcherNodelet::connectCb, this);
	// Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
	boost::lock_guard < boost::mutex > lock(connect_mutex_);
	//pub_disparity_ = nh.advertise<DisparityImage>("surf_cloud", 1, connect_cb, connect_cb);

	image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
	pub_image_ = it_->advertise("image_matches", 1);
	sub_l_image_.subscribe(*it_, "left/image_rect", 1, hints);
	sub_l_info_.subscribe(nh, "left/camera_info", 1);
	sub_r_image_.subscribe(*it_, "right/image_rect", 1, hints);
	sub_r_info_.subscribe(nh, "right/camera_info", 1);

	cloud_pub = nh.advertise<PointCloud>("cloud_matches", 1);
}
/*
 // Handles (un)subscribing when clients (un)subscribe
 void FeatureMatcherNodelet::connectCb() {
 boost::lock_guard < boost::mutex > lock(connect_mutex_);
 if (pub_matches_.getNumSubscribers() == 0) {
 sub_l_image_.unsubscribe();
 sub_l_info_.unsubscribe();
 sub_r_image_.unsubscribe();
 sub_r_info_.unsubscribe();
 } else if (!sub_l_image_.getSubscriber()) {
 ros::NodeHandle &nh = getNodeHandle();
 // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
 /// @todo Allow remapping left, right?
 image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
 sub_l_image_.subscribe(*it_, "left/image_rect", 1, hints);
 sub_l_info_.subscribe(nh, "left/camera_info", 1);
 sub_r_image_.subscribe(*it_, "right/image_rect", 1, hints);
 sub_r_info_.subscribe(nh, "right/camera_info", 1);
 }
 }
 */
void FeatureMatcherNodelet::imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg, const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg) {

	// Update the camera model
	model_.fromCameraInfo(l_info_msg, r_info_msg);

	CvImageConstPtr img1;
	CvImageConstPtr img2;

	try {
		img1 = toCvShare(l_image_msg, "mono8");
		img2 = toCvShare(r_image_msg, "mono8");
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// detecting keypoints
	SurfFeatureDetector detector(400);
	vector<KeyPoint> keypoints1, keypoints2;
	detector.detect(img1->image, keypoints1);
	detector.detect(img2->image, keypoints2);

	// computing descriptors
	SurfDescriptorExtractor extractor;
	Mat descriptors1, descriptors2;
	extractor.compute(img1->image, keypoints1, descriptors1);
	extractor.compute(img2->image, keypoints2, descriptors2);

	// matching descriptors
	BFMatcher matcher(NORM_L2);
	vector<DMatch> matches;
	matcher.match(descriptors1, descriptors2, matches);

	vector<DMatch> matches_filter;

	Point3d xyz;

	cloud_msg.header.stamp = ros::Time::now();
	cloud_msg.header.frame_id = "/stereo_camera";
	//cloud_msg->height = 320;
	cloud_msg.points.clear();

	for (size_t i = 0; i < matches.size(); i++) {
		int i1 = matches[i].queryIdx;
		int i2 = matches[i].trainIdx;

		const KeyPoint &kp1 = keypoints1[i1], &kp2 = keypoints2[i2];

		if (abs(kp1.pt.y - kp2.pt.y) < 10) {

			matches_filter.push_back(matches[i]);

			model_.projectDisparityTo3d(kp1.pt, matches[i].distance, xyz);

			//std::cout << "(u,v)(x,y,z)" << kp1.pt.x << " " << kp1.pt.y << " : " << xyz.x << " " <<xyz.y << " " << xyz.z << std::endl;

			cloud_msg.points.push_back(pcl::PointXYZ(xyz.x / 16.0, xyz.y / 16.0, xyz.z / 16.0));

		}

	}

	//cloud_msg->width = 240;

	//std::cout << "matches count:" << matches_filter.size() << std::endl;

	Mat img_matches;
	drawMatches(img1->image, keypoints1, img2->image, keypoints2, matches_filter, img_matches);

	//sensor_msgs::Image omsg;
	CvImageConstPtr img_out;

	cv_bridge::CvImage cvi;
	//cvi.header.stamp = ros::Time::now();
	//cvi.header.frame_id = "camera";
	cvi.encoding = "rgb8";
	cvi.image = img_matches;

	pub_image_.publish(cvi.toImageMsg());

	cloud_pub.publish(cloud_msg);

}

}
// Register nodelet
#include <pluginlib/class_list_macros.h>
//PLUGINLIB_DECLARE_CLASS(lrm_stereo, feature_matcher, lrm_stereo::FeatureMatcherNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(lrm_stereo::FeatureMatcherNodelet, nodelet::Nodelet)
