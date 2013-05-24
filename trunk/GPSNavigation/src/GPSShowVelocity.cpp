#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

using namespace std;

double _rate = 1;
double _x = 0;
double _y = 0;
ros::Publisher _vel_pub;
ros::Time _t(0);

std_msgs::Float32 _msg;

void callback_pose(const geometry_msgs::PoseStampedPtr pose)
{
	//if(_t.isValid()) {

		ros::Duration t = pose->header.stamp - _t;

		if(t.toSec()>_rate) {
			double x = pose->pose.position.x - _x;
			double y = pose->pose.position.y - _y;

			double dist = sqrt(x*x + y*y);

			double vel_ms = dist/t.toSec() ;

			//ROS_DEBUG_STREAM(vel_ms << " m/s (" << vel_ms*3.6 << " km/h)");
			ROS_INFO_STREAM(vel_ms << " m/s (" << vel_ms*3.6 << " km/h)");

			_msg.data = vel_ms;
			_vel_pub.publish(_msg);

			_t = pose->header.stamp;
			_x = pose->pose.position.x;
			_y = pose->pose.position.y;
		}
		//running a bag in a loop, the time goes back
		else if (t.toSec()<0) {
			_t = pose->header.stamp;
		}
	//}
	//else {
	//	ROS_INFO_STREAM("invalid " << _t);
	//	_t = pose->header.stamp;
	//}
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "GPSShowVelocity");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    nh_priv.param("rate", _rate, 0.5);

    ros::Subscriber sub = nh.subscribe("pose", 1, callback_pose);
    _vel_pub = nh.advertise<std_msgs::Float32>(nh_priv.getNamespace() + "/velocity", 1);
    //_vel_pub = nh.advertise<std_msgs::Float32>("followme/velocity", 1);

    //_msg.header.frame_id = "velocity";
    ROS_INFO_STREAM("show velocity start spinning...");

    ros::spin();

    return 0;
}
