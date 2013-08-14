#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"
#include "sensor_msgs/LaserScan.h"
#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include "dynamic_reconfigure/server.h"

double angle_min_ , angle_max_ , angle_increment_ ,
   scan_time_ , range_min_ , range_max_ , range_min_sq_ ;
   
   
std::string output_frame_id_;

ros::Publisher pub_;

const double offset = M_PI/2;

void scanCB(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
   
    sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
    output->header = cloud->header;
    output->angle_min = angle_min_;
    output->angle_max = angle_max_;
    output->angle_increment = angle_increment_;
    output->time_increment = 0.0;
    output->scan_time = scan_time_;
    output->range_min = range_min_;
    output->range_max = range_max_;

    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, output->range_max+0.1);

    pcl::PointCloud<pcl::PointXYZ> cloud_pcl;
    pcl::fromROSMsg (*cloud, cloud_pcl);
    
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud_pcl.begin(); it != cloud_pcl.end(); ++it)
    {
      const float &x = it->x;
      const float &y = it->y;
      const float &z = it->z;

      if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
      {
        ROS_INFO("rejected for nan in point(%f, %f, %f)\n", x, y, z);
        continue;
      }

      double range_sq = y*y+x*x;
      if (range_sq < range_min_sq_) {
//        ROS_INFO("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
        continue;
      }

      double angle = -atan2(x, y) + offset;
      if (angle < output->angle_min)
      {
//         ROS_INFO("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
         angle += 2*M_PI;
//        continue;
      }
      if (angle > output->angle_max)
      {
//         ROS_INFO("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
         angle -= 2*M_PI;
      }

      int index = (angle - output->angle_min) / output->angle_increment;


      if (output->ranges[index] * output->ranges[index] > range_sq)
        output->ranges[index] = sqrt(range_sq);
      }

    pub_.publish(output);
}
   
int main(int argc, char** argv) {

// funcao para definir o nome do node (programa)  
   ros::init(argc, argv, "cloud_to_scan");

   ros::NodeHandle nh_;

   // essa funcao substitui o nome de image pelo argumento passado.
   std::string cloud_topic = nh_.resolveName("cloud");    
      
   nh_.param("angle_min", angle_min_, -M_PI);
   nh_.param("angle_max", angle_max_, M_PI);
   nh_.param("angle_increment", angle_increment_, M_PI/180.0/1.0);
   nh_.param("scan_time", scan_time_, 1.0/20.0);
   nh_.param("range_min", range_min_, 0.0);
   nh_.param("range_max", range_max_, 30.0);
   
   range_min_sq_ = range_min_ * range_min_;
   
   ros::Subscriber sub_ = nh_.subscribe(
      cloud_topic, 10, &scanCB);

   pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
   
   ros::spin();
   
}
