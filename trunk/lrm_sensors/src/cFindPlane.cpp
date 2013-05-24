#include "cFindPlane.h"


cFindPlane::cFindPlane(float _threshold, int _maxIterations, Eigen::Vector3f _axis, float _epsThreshold) :
	threshold(_threshold), maxIterations(_maxIterations), axis(_axis), epsThreshold(_epsThreshold)
{
}


void cFindPlane::calculatePlane(PointCloud& pointcloud, float &translatex, float &translatez, float &angleScalar)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud = this->filter(pointcloud);

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxIterations);
	seg.setDistanceThreshold(threshold);
	seg.setAxis(axis);
	seg.setEpsAngle(epsThreshold);
	seg.setInputCloud(filteredPointCloud);


	seg.segment(inliers, coefficients); //Calcula o plano

	std::cout << "Numero de inliniers" << inliers.indices.size() << std::endl;

	int indice = inliers.indices.size() - 500;

	if (indice < 0)
	{
		std::cout << "Numero minimo de inliers nao atingido" << std::endl;
		return;
	}

	translatex = 0.0f;
	translatez = 0.0f;

	for (int i = indice; i < indice + 20; i++)                       //Calculate the mean of some points that belongs to the plane
	{
		translatex += filteredPointCloud->points[inliers.indices[i]].x;
		translatez += filteredPointCloud->points[inliers.indices[i]].z;
	}

	translatex /= 20.0f;
	translatez /= 20.0f;

	std::cout << "tx: " << translatex << "  tz: " << translatez << std::endl;


	float zRotate = (-coefficients.values[0]*25.0f -coefficients.values[3]) / coefficients.values[2]; //Calculate the Z value on the plane when X == 50

	angleScalar = atan((zRotate - translatez)/ 25.0f);

	std::cout << "angles: " << angleScalar << std::endl;
}



pcl::PointCloud<pcl::PointXYZ>::Ptr cFindPlane::filter(PointCloud& pointcloud)
{
	PointCloud::Ptr filteredPointCloud(PointCloud);

	for (unsigned int i = 0; i < pointcloud.height; i++)
	{
		for (unsigned int j = 0; j < pointcloud.width; j++)
		{
			int index = i * pointcloud.width + j;
			if (pointcloud.points[index].x > 0.0f)
				filteredPointCloud->points.push_back(pcl::PointXYZ(pointcloud.points[index].x, pointcloud.points[index].y,
						pointcloud.points[index].z)); //adiciona apenas pontos validos a nuvem de pontos
		}
	}

	return filteredPointCloud;
}


void cFindPlane::alignPointCloud(PointCloud& pointcloud, float translatex, float translatez, float angleScalar)
{

	Eigen::AngleAxis<float> angleY(angleScalar, Eigen::Vector3f(0, 1, 0));

	for (unsigned int i = 0; i < pointcloud->height; i++)
	{
		for (unsigned int j = 0; j < pointcloud->width; j++)
		{
			int index = i * pointcloud->width + j;
			Eigen::Vector3f p(pointcloud->points[index].x - translatex, pointcloud->points[index].y, pointcloud->points[index].z - translatez);
			Eigen::Vector3f newp = angleY * p;
			pointcloud->points[index].x = newp(0, 0) + translatex;
			pointcloud->points[index].y = newp(1, 0);
			pointcloud->points[index].z = newp(2, 0);
		}
	}
}
