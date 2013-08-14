/**
 * @file cFindPlane.h
 * @author  Caio Cesar Teodoro Mendes <caiom@icmc.usp.br>
 * @version 1.0
 *
 * @section LICENSE
 *
 * JACTO license.
 *
 * @section DESCRIPTION
 *
 * cFindPlane is a class used to find a plane and align the input point cloud with it.
 */


#ifndef _CFINDPLANE_H_
#define _CFINDPLANE_H_

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <Eigen/Core>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>

/**
* \brief cFindPlane is a class used to find a plane and align the input point cloud with it.
*
* \code
*
* float translatey, translatez, angle;
*
* cFindPlane findplane(0.1f, 2000, Eigen::Vector3f(0, 0, 1), 0.7f);
*
* findplane.calculatePlane(pointcloud, translatey, translatez, angle); //Encontra um plano e guarda os parametros
* findplane.alignPointCloud(pointcloud, translatey, translatez, angle); //Alinha a point cloud baseando-se nos parametros encontrados
*
* \endcode
*
*
*/

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class cFindPlane
{
	float threshold; /**< Threshold, in meters, for two points to belong to the same plane.  */
	int maxIterations; /**< Max RANSAC paradigm iteractions.  */
	Eigen::Vector3f axis; /**< The axis the plane need to be perpendicular to.  */
	float epsThreshold;  /**< Threshold, in radians, between the axis the the plane.  */

	pcl::ModelCoefficients coefficients; /**< Parameters of the plane in the format: ax + by + cz + d = 0.  */
	pcl::PointIndices inliers;   /**< Indices of the points that belongs to the defined point cloud.  */

	pcl::SACSegmentation<pcl::PointXYZ> seg;  /**< PLC library segmentation object.  */


	/**
	* \brief Filters the point cloud, removing invalid points.
	*
	* @param pointcloud The point cloud to be filtered.
	* @return The filtered point cloud.
	*/
	PointCloud::Ptr filter(PointCloud& pointcloud);

public:

	/**
	* \brief Class constructor.
	*
	* \param _threshold Threshold, in meters, to find the plane.
	* \param _maxIterations Max interations of the RANSAC paradgin to find the plane.
	* \param _axis The axis for the plane to be perpendicular to.
	* \param _epsThreshold Angle threshold, in radians, between the plane and the axis.
	* \return The filtered point cloud.
	*/
	cFindPlane(float _threshold, int _maxIterations, Eigen::Vector3f _axis, float _epsThreshold);


	/**
	* \brief Calculate plane parameters on the given point cloud that is parallel to the axis atribute.
	*
	* \param pointcloud The point cloud in witch find the plane.
	* \param translatey[out] Mean Y coordinate value of some of the points that composes the plane.
	* \param translatez[out] Mean Z coordinate value of some of the points that composes the plane.
	* \param angleScalar[out] Angle between the plane and the X axis.
	* \return The filtered point cloud.
	*/
	void calculatePlane(PointCloud& pointcloud, float &translatey, float &translatez, float &angleScalar);


	/**
	* \brief Aligns the point cloud with the estimated plane, given two translations and a rotation angle.
	*
	* @param pointcloud The point cloud to be aligned.
	* @param translatey Translation on Y axis.
	* @param translatez Translation on Z axis.
	* @param angleScalar The rotation angle on the Y axis.
	*/
	void alignPointCloud(PointCloud& pointcloud, float translatey, float translatez, float angleScalar);

};

#endif
