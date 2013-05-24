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
 * @file viz_image_node.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 20, 2012
 *
 */

#include <ros/ros.h>
//#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
//#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void callback(const sensor_msgs::ImageConstPtr& msgs)
{
	//cv_bridge::CvImageConstPtr cp = cv_bridge::toCvShare(msgs, "rgb8");
	sensor_msgs::PointCloud2 output;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	//sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2());

	//cloud.toROSMsg(output)

	//output.header.frame_id = "/cloud";
	//output.header.stamp = ros::Time::now();

	/*
	output.width = msgs->width;
	output.height = msgs->height;

	output.data.resize(msgs->data.size());
	output.point_step = msgs->step;
	std::copy(msgs->data.begin(), msgs->data.end(),  output.data.begin());
	*/

	//memcpy(&output.data, msgs->data, output.data.size());
	//output.data[1].
	//msgs->data

	int ndx = 0;
	int idx;
	unsigned int i;
	unsigned int j;
	cloud.header.frame_id = "/cloud";
	cloud.header.stamp = ros::Time::now();

	double w = msgs->width;
	double h = msgs->height;
	int channel;

	cloud.width = msgs->width;
	cloud.height = msgs->height;
	cloud.points.resize(cloud.width * cloud.height);

	//ROS_INFO_STREAM("sz=" << msgs->data.size() << " ss=" << cloud.points.size());

	int channels = msgs->data.size() / cloud.points.size();

//#pragma omp for private(i, j, idx, ndx)
	for(i=0; i<msgs->height; i++) {
		for(j=0; j<msgs->width; j++) {
			idx = i*msgs->width*channels+(j*channels);
			//idx = j*msgs->height+i;

			//color = (msgs->data[idx] + msgs->data[idx+1] + msgs->data[idx+2]) / 3;
			if(channels>1) {
				channel = msgs->data[idx+1]; //Green
			}
			else {
				channel = msgs->data[idx];
			}
			//if(channel<250) {
				cloud.points[ndx].x = (j / 500.0);
				cloud.points[ndx].y = (i / 500.0);
				cloud.points[ndx].z = -channel / 512.0;
			//}
			ndx++;
		}
	}

	pcl::toROSMsg(cloud, output);

	// Publish the data
	pub.publish(output);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "viz_image_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	//image_transport::ImageTransport it(nh);

	//parvo_image_pub_ = it.advertise(nh_priv.getNamespace() + "/parvo/image_raw", 1);
	//magno_image_pub_ = it.advertise(nh_priv.getNamespace() + "/magno/image_raw", 1);

    ros::Subscriber sub = nh.subscribe("image_raw", 1, callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    //retina_ = 0;

    ros::spin();
}
