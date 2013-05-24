#include "cFindPlane.h"


cFindPlane::cFindPlane(float _threshold, int _maxIterations, Eigen::Vector3f _axis, float _epsThreshold):
	threshold(_threshold), maxIterations(_maxIterations), axis(_axis), epsThreshold(_epsThreshold)
{
}

void cFindPlane::calculatePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, float &translatey, float &translatez, float &angleScalar)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud = this->filter(pointcloud);

	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE/*pcl::SACMODEL_PLANE*/);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (maxIterations);
	seg.setDistanceThreshold (threshold);
	seg.setAxis(axis);
	seg.setEpsAngle(epsThreshold);
	seg.setInputCloud (filteredPointCloud);

	seg.segment (inliers, coefficients);           //Calcula o plano

	std::cout<< "Numero de inliniers " << inliers.indices.size () << std::endl;

	int indice = inliers.indices.size () - 500;

		if(indice < 0)
		{
			std::cout<< "Numero minimo de inliers nao atingido" << std::endl;
			return;
		}

		translatey = 0.0f;
		translatez = 0.0f;

		for(int i=indice; i<indice+20; i++)
		{
			translatey+= filteredPointCloud->points[inliers.indices[i]].y;
			translatez+= filteredPointCloud->points[inliers.indices[i]].z;
		}

		translatey /= 20.0f;
		translatez /= 20.0f;

		float yRotate = 0.0f;
		float zRotate = 0.0f;


		for(int i=0; i<50; i++)
		{
			yRotate+=filteredPointCloud->points[inliers.indices[i]].y - translatey;
			zRotate+=filteredPointCloud->points[inliers.indices[i]].z - translatez;
		}

		yRotate/=50.0f;
		zRotate/=50.0f;

		Eigen::Vector2f  v1(0, 1);
		Eigen::Vector2f  v2(yRotate, zRotate);

		v2.normalize();
		v1.normalize();

		angleScalar = v1.dot(v2);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cFindPlane::filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPointCloud(new pcl::PointCloud<pcl::PointXYZ>);

	for(unsigned int i=0; i<pointcloud->height; i++)
	{
		for(unsigned int j=0; j<pointcloud->width; j++)
		{
			int index = i*pointcloud->width + j;
			//if(pointcloud->points[index].z < 0.0f)
				filteredPointCloud->points.push_back(pcl::PointXYZ(pointcloud->points[index].x, pointcloud->points[index].y, pointcloud->points[index].z)); //filtra pontos no infinito
		}
	}

	return filteredPointCloud;
}

void cFindPlane::rotatePlaneToZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, float translatey, float translatez, float angleScalar)
{

	Eigen::AngleAxis<float> angleX(-acos(angleScalar), Eigen::Vector3f(1,0,0));

	for(unsigned int i=0; i<pointcloud->height; i++)         //translada e rotaciona a nuvem de pontos
	{
		for(unsigned int j=0; j<pointcloud->width; j++)
		{
			int index = i*pointcloud->width + j;
			Eigen::Vector3f p(pointcloud->points[index].x, pointcloud->points[index].y - translatey, pointcloud->points[index].z - translatez);
			Eigen::Vector3f newp = angleX*p;
			pointcloud->points[index].x = newp(0,0);
			pointcloud->points[index].y = newp(1,0);
			pointcloud->points[index].z = newp(2,0) - translatez;
		}
	}

	//std::cout << "rotated" << std::endl;

}
