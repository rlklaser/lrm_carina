#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "lrm_stereo/throttle_stereo.h"

namespace lrm_stereo
{

class ThrottleStereoNodelet: public nodelet::Nodelet
{
public:
//	StereoCameraThrottler* throttler_;
	virtual void onInit();
//	virtual ~ThrottleStereoNodelet();
private:
	boost::shared_ptr<StereoCameraThrottler> server_;
};

void ThrottleStereoNodelet::onInit()
{
	ros::NodeHandle& nh = getNodeHandle();
	ros::NodeHandle& private_nh = getPrivateNodeHandle();

	//throttler_ = new StereoCameraThrottler(nh, private_nh);
	server_.reset(new StereoCameraThrottler(nh, private_nh));
}

//ThrottleStereoNodelet::~ThrottleStereoNodelet()
//{
//	delete throttler_;
//}

}
// Register nodelet
PLUGINLIB_EXPORT_CLASS(lrm_stereo::ThrottleStereoNodelet, nodelet::Nodelet)
