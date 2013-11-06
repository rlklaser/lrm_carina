#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "lrm_stereo/throttle_stereo.h"

namespace lrm_stereo
{

class ThrottleStereoNodelet: public nodelet::Nodelet
{
public:
	virtual void onInit();
private:
	boost::shared_ptr<StereoCameraThrottler> server_;
};

void ThrottleStereoNodelet::onInit()
{
	ROS_WARN("throttle nodelet init");
	ros::NodeHandle& nh = getNodeHandle();
	ros::NodeHandle& private_nh = getPrivateNodeHandle();

	server_.reset(new StereoCameraThrottler(nh, private_nh));
}

}
// Register nodelet
PLUGINLIB_EXPORT_CLASS(lrm_stereo::ThrottleStereoNodelet, nodelet::Nodelet)
