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
 * @file zigzag_filter.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Sep 30, 2013
 *
 */

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <boost/assign.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>

#include <math.h>

using namespace boost::accumulators;

ros::Publisher pc_pub;
ros::Publisher pc_rem_pub;

sensor_msgs::PointCloud2 cloud_out;

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

	//if (pc_pub.getNumSubscribers() == 0)
	//	return;

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_rem_out;

	typedef pcl::PointCloud<pcl::PointXYZRGB>::iterator itrtor;
	//typedef cloud->data.iterator it;

	sensor_msgs::PointCloud2 msg_out;

	//cloud_out.header = msg->header;

	pcl::fromROSMsg(*msg, cloud);

	double min_x = 99999999;
	double max_x = 0;
	long qtd = 0;
	double x;
	double tot = 0;
	double mean = 0;
	double var = 0;
	double stdev = 0;
	double centered_x;

	itrtor end = cloud.points.end();
	for (itrtor itr = cloud.points.begin(); itr != end; ++itr )
	{
		qtd++;
		x = itr->x;
		tot += x;

		if(x>max_x) max_x = x;
		if(x<min_x) min_x = x;
	}

	tot = tot - (min_x * qtd);
	mean = tot / qtd;

	tot = 0;
	for (itrtor itr = cloud.points.begin(); itr != end; ++itr )
	{
		x = itr->x;
		centered_x = x-min_x;
		tot += (centered_x-mean)*(centered_x-mean);
	}

	var = tot / qtd;

	stdev = sqrt(var);

	qtd = 0;
	for (itrtor itr = cloud.points.begin(); itr != end; ++itr )
	{
		x = itr->x;
		centered_x = x-min_x;
		if( centered_x > (mean-stdev) && centered_x < (mean+stdev)) {
			cloud_out.points.push_back(*itr);
		}
		else {
			cloud_rem_out.points.push_back(*itr);
			qtd++;
		}
	}

	pcl::toROSMsg(cloud_out, msg_out);
	msg_out.header = msg->header;
	pc_pub.publish(msg_out);

	pcl::toROSMsg(cloud_rem_out, msg_out);
	msg_out.header = msg->header;
	pc_rem_pub.publish(msg_out);


	std::cout << "removed " << qtd << std::endl;

	//double sum;

	//BOOST_FOREACH(sensor_msgs::PointCloud2::Type point, msg->data)
	//{
    //
	//}

	/*
	 double sum = std::accumulate(std::begin(msg->points), std::end(msg->points), 0.0);
	 double m =  sum / msg->points.size();

	 double accum = 0.0;
	 std::for_each (std::begin(msg->points), std::end(msg->points), [&](const double d) {
	 accum += (d - m) * (d - m);
	 });

	 double stdev = sqrt(accum / (msg->points.size()-1));
	 */

	/*
	double v[msg->data.size()];
	for_each(msg->data.begin(), msg->data.end(), bind<void>(ref(acc), _1));

	accumulator_set<double, stats<tag::variance> > acc;
	for_each(v.begin(), v.end(), bind<void>(ref(acc), _1));
	cout << mean(acc) << endl;
	cout << sqrt(variance(acc)) << endl;
	*/

	//msg->data::iterator end = msg->data.end();
	//for (msg->data::iterator itr = cloud->data.begin(); itr != end; ++itr )
	{

	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "zigzag_filter_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	//tf_listener = new tf::TransformListener(nh, ros::Duration(30), true);

	//ros::Subscriber pc_sub = nh.subscribe("points_in", 1, pointcloudCallback);
	ros::Subscriber pc_sub = nh.subscribe("/cloud/points_cluster", 1, pointcloudCallback);

	pc_pub = nh.advertise<sensor_msgs::PointCloud2>(nh_priv.getNamespace() + "/points_out", 1);
	pc_rem_pub = nh.advertise<sensor_msgs::PointCloud2>(nh_priv.getNamespace() + "/points_out_removed", 1);

	//nh_priv.param<int>("frame_count", _nro_of_frames, 5);
	//nh_priv.param<bool>("incremental", _incremental, false);

	//ROS_INFO_STREAM("cloud sum within " << _nro_of_frames << " frames");

	ros::spin();

	//delete tf_listener;

	return 0;
}
