
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"

#include <boost/bind.hpp>


class DeclinationTransform : public tf::Transform {
public:
  void set_declination(double decl) {
    tf::Quaternion quat;
    quat.setEuler(decl, 0.0, 0.0);
    setRotation(quat); 
  }

  void msg(const std_msgs::Float32ConstPtr declination_msg)
  {
    set_declination(declination_msg->data);
  }
};


static void imu_to_frame(const tf::TransformListener& tf_listener,
                         std::string& tf_link,
                         const sensor_msgs::Imu& imu_in, 
                         sensor_msgs::Imu& imu_out)
{
  tf::Stamped<tf::Quaternion> orient_in, orient_out;
  tf::quaternionMsgToTF(imu_in.orientation, orient_in);
  orient_in.frame_id_ = imu_in.header.frame_id;
  tf_listener.transformQuaternion(tf_link, orient_in, orient_out);
  tf::quaternionTFToMsg(orient_out, imu_out.orientation);

  tf::Stamped<tf::Vector3> vel_in, vel_out;
  tf::vector3MsgToTF(imu_in.angular_velocity, vel_in);
  vel_in.frame_id_ = imu_in.header.frame_id;
  tf_listener.transformVector(tf_link, vel_in, vel_out);
  tf::vector3TFToMsg(vel_out, imu_out.angular_velocity);
  
  tf::Stamped<tf::Vector3> accel_in, accel_out;
  tf::vector3MsgToTF(imu_in.linear_acceleration, accel_in);
  accel_in.frame_id_ = imu_in.header.frame_id;
  tf_listener.transformVector(tf_link, accel_in, accel_out);
  tf::vector3TFToMsg(accel_out, imu_out.linear_acceleration);

  imu_out.header.stamp = imu_in.header.stamp;
  imu_out.header.frame_id = tf_link;
}


static void handle_imu(const sensor_msgs::ImuConstPtr& imu_ptr,
                       const ros::Publisher& pub_imu,
                       const tf::TransformListener& tf_listener,
                       const DeclinationTransform& transform,
                       std::string tf_link)
{
  // Must move imu into base_link frame before applying declination.
  sensor_msgs::Imu imu;
  imu_to_frame(tf_listener, tf_link, *imu_ptr, imu);

  // Rotate orientation by declination amount.
  tf::Quaternion orient;
  tf::quaternionMsgToTF(imu.orientation, orient); 
  orient = transform * orient;
  tf::quaternionTFToMsg(orient, imu.orientation);

  pub_imu.publish(imu);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "apply_declination_to_imu");

  ros::NodeHandle np("~");
  std::string tf_link;
  np.param<std::string>("tf_link", tf_link, "base_link");

  double declination_rads;
  np.param<double>("default", declination_rads, 0.0);

  DeclinationTransform declination_transform;
  declination_transform.set_declination(declination_rads);

  ros::NodeHandle n;
  tf::TransformListener tf_listener(n);
  ros::Subscriber sub = n.subscribe("declination", 5, 
      &DeclinationTransform::msg, &declination_transform);
  ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("data_decl", 5);
  ros::Subscriber sub_enu = n.subscribe<sensor_msgs::Imu>("data", 5,
      boost::bind(handle_imu, _1, boost::ref(pub_imu), boost::ref(tf_listener), 
                  boost::ref(declination_transform), tf_link));

  ros::spin();
  return 0;
}

