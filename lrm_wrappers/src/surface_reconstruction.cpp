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
 * @file surface_reconstruction.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date May 20, 2013
 *
 */

#pragma clang diagnostic ignored "-Woverloaded-virtual"
#pragma clang diagnostic ignored "-Winvalid-offsetof"
#pragma clang diagnostic ignored "-Wextra-tokens"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//#include <pcl_ros/transforms.h>
//#include <pcl_ros/point_cloud.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>

ros::Publisher pc_pub_gd;
ros::Publisher pc_pub_ngd;

ros::Publisher cf_pub;

sensor_msgs::PointCloud2 ground_points;
sensor_msgs::PointCloud2 no_ground_points;

using namespace pcl;

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
	//pcl::PointCloud<pcl::PointXYZ> cloud;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	pcl::fromROSMsg(*msg, *cloud_filtered);

	MovingLeastSquares < PointXYZ, PointXYZ > mls;
	mls.setInputCloud(cloud_filtered);
	mls.setSearchRadius(0.01);
	mls.setPolynomialFit(true);
	mls.setPolynomialOrder(2);
	mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius(0.005);
	mls.setUpsamplingStepSize(0.003);
	PointCloud<PointXYZ>::Ptr cloud_smoothed(new PointCloud<PointXYZ>());
	mls.process(*cloud_smoothed);


	pcl::toROSMsg(*cloud_smoothed, ground_points);
	pc_pub_gd.publish(ground_points);


	NormalEstimation < PointXYZ, Normal > ne;
	//ne.setNumberOfThreads(2);
	ne.setInputCloud(cloud_smoothed);
	ne.setRadiusSearch(0.01);
	Eigen::Vector4f centroid;
	compute3DCentroid(*cloud_smoothed, centroid);
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
	PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
	ne.compute(*cloud_normals);
	for (size_t i = 0; i < cloud_normals->size(); ++i) {
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}
	//PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
	//concatenateFields(*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);

	pcl::toROSMsg(*cloud_normals, no_ground_points);
	pc_pub_ngd.publish(no_ground_points);

	/*
	Poisson < PointNormal > poisson;
	poisson.setDepth(9);
	poisson.setInputCloud(cloud_smoothed_normals);
	PolygonMesh mesh;
	poisson.reconstruct(mesh);
	*/

	//io::saveVTKFile ("/home/rlkaser/mesh.vtk", mesh);


	pcl::PointCloud<pcl::PointXYZ> cloud_out;

//	extract_gd.filter(cloud_out);
//	pcl::toROSMsg(cloud_out, ground_points);
//	pc_pub_gd.publish(ground_points);

//	extract_ngd.filter(cloud_out);
//	pcl::toROSMsg(cloud_out, no_ground_points);
//	pc_pub_ngd.publish(no_ground_points);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "surface_reconstruction");
	ros::NodeHandle n;

	//ros::Subscriber pc_sub = n.subscribe("/stereo/points2", 5, pointcloudCallback);

	ros::Subscriber pc_sub = n.subscribe("ground_points", 5, pointcloudCallback);


	pc_pub_gd = n.advertise<sensor_msgs::PointCloud2>("ground_points_surface", 5);
	pc_pub_ngd = n.advertise<sensor_msgs::PointCloud2>("no_ground_points_surface", 5);
	cf_pub = n.advertise<pcl::ModelCoefficients>("ground", 1);

	ros::spin();
}

