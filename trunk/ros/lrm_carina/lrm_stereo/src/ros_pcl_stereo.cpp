#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>

#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <stereo_image_proc/DisparityConfig.h>
#include <dynamic_reconfigure/server.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/stereo/stereo_matching.h>

namespace lrm_stereo
{

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;

class RosPclStereoNodelet: public nodelet::Nodelet
{
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
	ros::Publisher pub_points2_;

	// Dynamic reconfigure
	boost::recursive_mutex config_mutex_;
	typedef stereo_image_proc::DisparityConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
	boost::shared_ptr<ReconfigureServer> reconfigure_server_;

	// Processing state (note: only safe because we're single-threaded!)
	image_geometry::StereoCameraModel model_;
	pcl::AdaptiveCostSOStereoMatching stereo;

	virtual void onInit();
	void connectCb();
	void imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg, const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg);
	void configCb(Config &config, uint32_t level);
};

void RosPclStereoNodelet::onInit()
{
	ros::NodeHandle &nh = getNodeHandle();
	ros::NodeHandle &private_nh = getPrivateNodeHandle();

	it_.reset(new image_transport::ImageTransport(nh));

	ROS_INFO_STREAM("loading pcl stereo nodelet.....");

	// Synchronize inputs. Topic subscriptions happen on demand in the connection
	// callback. Optionally do approximate synchronization.
	int queue_size;
	private_nh.param("queue_size", queue_size, 5);
	bool approx;
	private_nh.param("approximate_sync", approx, false);
	if (approx)
	{
		approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size), sub_l_image_, sub_l_info_, sub_r_image_, sub_r_info_));
		approximate_sync_->registerCallback(boost::bind(&RosPclStereoNodelet::imageCb, this, _1, _2, _3, _4));
	}
	else
	{
		exact_sync_.reset(new ExactSync(ExactPolicy(queue_size), sub_l_image_, sub_l_info_, sub_r_image_, sub_r_info_));
		exact_sync_->registerCallback(boost::bind(&RosPclStereoNodelet::imageCb, this, _1, _2, _3, _4));
	}

	// Set up dynamic reconfiguration
	ReconfigureServer::CallbackType f = boost::bind(&RosPclStereoNodelet::configCb, this, _1, _2);
	reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
	reconfigure_server_->setCallback(f);

	stereo.setMaxDisparity(64);
	stereo.setXOffset(0);
	stereo.setRadius(5);
	stereo.setSmoothWeak(20);
	stereo.setSmoothStrong(100);
	stereo.setGammaC(25);
	stereo.setGammaS(10);
	stereo.setRatioFilter(20);
	stereo.setPeakFilter(0);
	stereo.setLeftRightCheck(true);
	stereo.setLeftRightCheckThreshold(1);
	stereo.setPreProcessing(true);

	// Monitor whether anyone is subscribed to the output
	ros::SubscriberStatusCallback connect_cb = boost::bind(&RosPclStereoNodelet::connectCb, this);
	// Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
	boost::lock_guard < boost::mutex > lock(connect_mutex_);
	//pub_disparity_ = nh.advertise<DisparityImage>("disparity", 1, connect_cb, connect_cb);
	pub_points2_ = nh.advertise<PointCloud2>("pcl_points", 1, connect_cb, connect_cb);

	ROS_INFO_STREAM("pcl stereo nodelet loaded.");
}

// Handles (un)subscribing when clients (un)subscribe
void RosPclStereoNodelet::connectCb()
{
	boost::lock_guard < boost::mutex > lock(connect_mutex_);
	if (pub_points2_.getNumSubscribers() == 0)
	{
		sub_l_image_.unsubscribe();
		sub_l_info_.unsubscribe();
		sub_r_image_.unsubscribe();
		sub_r_info_.unsubscribe();
	}
	else
	{
		ROS_INFO_STREAM("will subscribe....");

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

void RosPclStereoNodelet::imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg, const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg)
{
	// Update the camera model
	model_.fromCameraInfo(l_info_msg, r_info_msg);

	//ROS_INFO_STREAM("callback...");

	/*
	 pcl::PointCloud<pcl::RGB>::Ptr point_cloud_r(new pcl::PointCloud<pcl::RGB>);
	 pcl::PointCloud<pcl::RGB>::Ptr point_cloud_l(new pcl::PointCloud<pcl::RGB>);
	 {

	 {
	 uchar pr, pg, pb;

	 for (unsigned int i = 0; i < l_image_msg->height; i++) {
	 uint8_t* rgb_ptr = const_cast<uint8_t*>(&l_image_msg->data[i]);
	 for (unsigned int j = 0; j < l_image_msg->width; j++) {
	 //Get RGB info
	 pb = rgb_ptr[3 * j];
	 pg = rgb_ptr[3 * j + 1];
	 pr = rgb_ptr[3 * j + 2];

	 //Insert info into point cloud structure
	 pcl::RGB point;
	 uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
	 point.rgb = *reinterpret_cast<float*>(&rgb);
	 point_cloud_l->points.push_back(point);
	 }
	 }

	 }

	 {
	 uchar pr, pg, pb;

	 for (unsigned int i = 0; i < r_image_msg->height; i++) {
	 uint8_t* rgb_ptr = const_cast<uint8_t*>(&r_image_msg->data[i]);
	 for (unsigned int j = 0; j < r_image_msg->width; j++) {
	 //Get RGB info
	 pb = rgb_ptr[3 * j];
	 pg = rgb_ptr[3 * j + 1];
	 pr = rgb_ptr[3 * j + 2];

	 //Insert info into point cloud structure
	 pcl::RGB point;
	 uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
	 point.rgb = *reinterpret_cast<float*>(&rgb);
	 point_cloud_r->points.push_back(point);
	 }
	 }

	 }

	 }
	 stereo.compute(*point_cloud_l, *point_cloud_r);
	 */

	unsigned char* left_ptr = (unsigned char*) &l_image_msg->data[0];
	unsigned char* right_ptr = (unsigned char*) &r_image_msg->data[0];

	stereo.compute(left_ptr, right_ptr, l_image_msg->width, l_image_msg->height);

	//stereo.medianFilter(4);

	// Fill in new PointCloud2 message (2D image-like layout)
	PointCloud2Ptr points_msg = boost::make_shared<PointCloud2>();
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_out(new pcl::PointCloud<pcl::PointXYZ>);

	stereo.getPointCloud(
			model_.left().projectionMatrix()[0][2],
			model_.left().projectionMatrix()[1][2],
			model_.left().projectionMatrix()[0][0],
			model_.baseline(), point_out);

	//std::cout << "points: " << point_out->points.size();

	pcl::toROSMsg(*point_out, *points_msg);

	points_msg->header.frame_id = "/stereo_camera";
	points_msg->header.stamp = ros::Time::now();

	pub_points2_.publish(points_msg);
}

void RosPclStereoNodelet::configCb(Config &config, uint32_t level)
{

}

} // namespace ros_pcl_stereo

// Register nodelet
#include <pluginlib/class_list_macros.h>
//PLUGINLIB_DECLARE_CLASS(ros_pcl_stereo, points2, ros_pcl_stereo::RosPclStereoNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(lrm_stereo::RosPclStereoNodelet, nodelet::Nodelet)
