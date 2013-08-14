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
 * @file cloud_sum.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Jul 1, 2013
 *
 */

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <boost/make_shared.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//convenient structure to handle our pointclouds
struct PCD {
	PointCloud::Ptr cloud;
	std::string f_name;

	PCD() :
			cloud(new PointCloud) {
	}
	;
};

struct PCDComparator {
	bool operator ()(const PCD& p1, const PCD& p2) {
		return (p1.f_name < p2.f_name);
	}
};

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation: public pcl::PointRepresentation<PointNormalT> {
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
	MyPointRepresentation() {
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointNormalT &p, float * out) const {
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

ros::Publisher pc_pub;

sensor_msgs::PointCloud2 cloud_out;
PointCloud cloud_tot;
int frame_count;
tf::TransformListener* tf_listener;

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
 * \param cloud_src the source PointCloud
 * \param cloud_tgt the target PointCloud
 * \param output the resultant aligned source PointCloud
 * \param final_transform the resultant transform between source and target
 */
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false) {
	//
	// Downsample for consistency and speed
	// \note enable this for large datasets
	PointCloud::Ptr src(new PointCloud);
	PointCloud::Ptr tgt(new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	if (downsample) {
		grid.setLeafSize(0.05, 0.05, 0.05);
		grid.setInputCloud(cloud_src);
		grid.filter(*src);

		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	} else {
		src = cloud_src;
		tgt = cloud_tgt;
	}

	// Compute surface normals and curvature
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

	pcl::NormalEstimation<PointT, PointNormalT> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);

	norm_est.setInputCloud(src);
	norm_est.compute(*points_with_normals_src);
	pcl::copyPointCloud(*src, *points_with_normals_src);

	norm_est.setInputCloud(tgt);
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	//
	// Align
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	reg.setTransformationEpsilon(1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance(0.1);
	// Set the point representation
	reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

	reg.setInputCloud(points_with_normals_src);
	reg.setInputTarget(points_with_normals_tgt);

	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src;

	reg.setMaximumIterations(30);
	/*
	 reg.setMaximumIterations(2);
	 for (int i = 0; i < 30; ++i) {
	 PCL_INFO("Iteration Nr. %d.\n", i);

	 // save cloud for visualization purpose
	 points_with_normals_src = reg_result;

	 // Estimate
	 reg.setInputCloud(points_with_normals_src);
	 reg.align(*reg_result);

	 //accumulate transformation between each Iteration
	 Ti = reg.getFinalTransformation() * Ti;

	 //if the difference between this transformation and the previous one
	 //is smaller than the threshold, refine the process by reducing
	 //the maximal correspondence distance
	 if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
	 reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

	 prev = reg.getLastIncrementalTransformation();

	 // visualize current state
	 showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	 }
	 */

	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	//
	// Transform target back in source frame
	pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

	/*
	 p->removePointCloud("source");
	 p->removePointCloud("target");

	 PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
	 PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
	 p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
	 p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);

	 PCL_INFO("Press q to continue the registration.\n");
	 p->spin();

	 p->removePointCloud("source");
	 p->removePointCloud("target");
	 */

	//add the source to the transformed target
	*output += *cloud_src;

	final_transform = targetToSource;
}

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
	//pcl::PointCloud<pcl::PointXYZRGB> cloud;
	PointCloud cloud;
	pcl::fromROSMsg(*msg, cloud);

	tf::StampedTransform sensorToWorldTf;
	try {
		tf_listener->waitForTransform("/map", msg->header.frame_id, msg->header.stamp, ros::Duration(2.0));
		tf_listener->lookupTransform("/map", msg->header.frame_id, msg->header.stamp, sensorToWorldTf);
	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
		return;
	}

	Eigen::Matrix4f sensorToWorld;
	pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

	pcl::transformPointCloud(cloud, cloud, sensorToWorld);

	if (frame_count == 0) {
		cloud_tot += cloud;
	} else {
		PointCloud::Ptr result(new PointCloud), source, target;
		Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

		source = boost::make_shared < PointCloud > (cloud);
		target = boost::make_shared < PointCloud > (cloud_tot);

		// Add visualization data
		//showCloudsLeft(source, target);

		PointCloud::Ptr temp(new PointCloud);
		//PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[i - 1].f_name.c_str(), source->points.size(), data[i].f_name.c_str(), target->points.size());
		pairAlign(source, target, temp, pairTransform, false);

		//transform current pair into the global transform
		pcl::transformPointCloud(*temp, *result, GlobalTransform);

		//update the global transform
		GlobalTransform = pairTransform * GlobalTransform;
	}

	frame_count++;

	if (frame_count == 5) {
		frame_count = 0;
		pcl::toROSMsg(cloud_tot, cloud_out);

		cloud_out.header.frame_id = "/map";
		cloud_out.header.stamp = ros::Time::now();

		pc_pub.publish(cloud_out);
		cloud_tot.clear();
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "cloud_register_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	cloud_tot.clear();
	frame_count = 0;

	tf_listener = new tf::TransformListener();

    ros::Subscriber pc_sub = nh.subscribe("/stereo/points2", 1, pointcloudCallback);
	pc_pub = nh.advertise<sensor_msgs::PointCloud2>(nh_priv.getNamespace() + "/points_out", 1);

	ros::spin();

	delete tf_listener;

	return (0);
}
