#include "cObstacleDetection.h"
#include <pcl/sample_consensus/sac_model_plane.h>

#define VSIZE 240
#define HSIZE 320

#define HMIN 0.1f
#define HMAX 0.4f
#define ALPHAMAX 45

#define TOTALSIZE (HSIZE*VSIZE*3)
#define SIZEDIM (HSIZE*VSIZE)

#define COSALPHA 0.7 //cos(ALPHAMAX * PI / 180)

void detectObsCPU(float * img, float * data, int totalSizeV, int totalSizeH, int localSizeV, int localSizeH);

cObstacleDetection::cObstacleDetection(int _sizeX, int _sizeY, float _threshold, float _confiance) :
		threshold(_threshold), confiance(_confiance)
{
	data = new float[VSIZE * HSIZE * 3];
	fImg = new float[VSIZE * HSIZE];

	totalSizeV = VSIZE;
	totalSizeH = HSIZE;
	localSizeV = 1;//30;
	localSizeH = 1;//31;

	beginH = 0;//30;
	beginV = 0;//30;

#ifdef OPENCL
	vector<cl::Platform> platformList;

	cl::Platform::get(&platformList);

	std::string platformVendor;
	platformList[0].getInfo((cl_platform_info) CL_PLATFORM_VENDOR, &platformVendor);

	cl_context_properties cprops[3] =
	{ CL_CONTEXT_PLATFORM, (cl_context_properties) (platformList[0])(), 0 };

	//criacao do contexto, primeira passo a ser dado

	context = new cl::Context(CL_DEVICE_TYPE_GPU, cprops, NULL, NULL);

	//criando um buffer, especifico para um dado contexto

	dataBuffer = new cl::Buffer(*context, CL_MEM_READ_ONLY, TOTALSIZE * sizeof(float));
	imgBuffer = new cl::Buffer(*context, CL_MEM_READ_WRITE, SIZEDIM * sizeof(float));

	//Criando o device, especifico para compilacao e execucao de kernel

	devices = context->getInfo<CL_CONTEXT_DEVICES>();

	//Criacao e compilacao de um programa

	std::ifstream file("prog1_kernel.cl");
	string prog(std::istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));
	cl::Program::Sources source(1, std::make_pair(prog.c_str(), prog.length() + 1));

	cl::Program program(*context, source);
	program.build(devices);

	//criacao de kernels a partir do programa compilado

	kernel = new cl::Kernel(program, "detectC");

	kernel->setArg(0, *dataBuffer);
	kernel->setArg(1, *imgBuffer);

	//envia o kernel para o dispositivo

	queue = new cl::CommandQueue(*context, devices[0], 0);

#endif
}

void cObstacleDetection::detect(PointCloud::Ptr pointcloud, cv::Mat& img)
{

	for (int i = 0; i < totalSizeV; i++)
	{
		for (int j = 0; j < totalSizeH; j++)
		{
			fImg[i * totalSizeH + j] = 1;
		}
	}

	int pci, pcj;

	for (int k = 0; k < 3; k++)
	{
		for (int i = 0, pci = beginV; i < totalSizeV; i++, pci++)
		{
			for (int j = 0, pcj = beginH; j < totalSizeH; j++, pcj++)
			{
				if (k == 0)
					data[(k * totalSizeV * totalSizeH) + i * totalSizeH + j] = pointcloud->points[pci * pointcloud->width + pcj].x;
				else if (k == 1)
					data[(k * totalSizeV * totalSizeH) + i * totalSizeH + j] = pointcloud->points[pci * pointcloud->width + pcj].y;
				else
					data[(k * totalSizeV * totalSizeH) + i * totalSizeH + j] = pointcloud->points[pci * pointcloud->width + pcj].z;
			}
		}
	}

#ifdef OPENCL

	int rangeNDV = (totalSizeV - (localSizeV - 1)) * 2;
	int rangeNDH = (totalSizeH - (localSizeH - 1)) * localSizeH;


	queue->enqueueWriteBuffer(*dataBuffer, CL_TRUE, 0, TOTALSIZE * sizeof(float), data);
	queue->enqueueWriteBuffer(*imgBuffer, CL_TRUE, 0, SIZEDIM * sizeof(float), fImg);
	cl::Event event;

	queue->enqueueNDRangeKernel(*kernel, cl::NullRange, cl::NDRange(rangeNDV, rangeNDH), cl::NDRange(2, localSizeH), NULL, &event);
	//Aguarda a resposta e imprime o resultado

	event.wait();
	queue->enqueueReadBuffer(*imgBuffer, CL_TRUE, 0, SIZEDIM * sizeof(float), fImg);

#else

	detectObsCPU(fImg, data, totalSizeV, totalSizeH, localSizeV, localSizeH);

#endif

	return;

//classify image

}

void cObstacleDetection::generateImage(cv::Mat& img)
{
	int nav;

	int fi, fj;

	for (int iM = beginV, fi = 0; iM < beginV + totalSizeV; iM++, fi++)
	{
		cv::Vec3b * valPixel = img.ptr<cv::Vec3b>(iM);

		for (int jM = beginH, fj = 0; jM < beginH + totalSizeH; jM++, fj++)
		{
			nav = (int) fImg[fi * totalSizeH + fj];

			if (nav == 1) //navegavel
			{
				valPixel[jM][1] = 0;
				valPixel[jM][2] = 0;
			}
			else
			{
				valPixel[jM][0] = 0;
				valPixel[jM][1] = 0; //nao navegavel
			}

		}

	}
}



void detectObsCPU(float * img, float * data, int totalSizeV, int totalSizeH, int localSizeV, int localSizeH)
{
	if (localSizeH > totalSizeH || localSizeV > totalSizeV)
		std::cout << "Local > Total!!" << std::endl;
	if (localSizeH % 2 != 1)
		std::cout << "Tamanho horizontal par!!" << std::endl;

	int rangeVOut = totalSizeV - (localSizeV - 1);
	int rangeHOut = totalSizeH - (localSizeH - 1);

	int globalSize = totalSizeV * totalSizeH;

	for (int VOut = 0; VOut < rangeVOut; VOut++)
	{
		for (int HOut = 0; HOut < rangeHOut; HOut++)
		{

			int pivotV = VOut + (localSizeV - 1);
			int pivotH = HOut + (localSizeH - 1) / 2;

			float x = data[pivotV * totalSizeH + pivotH];
			float y = data[(pivotV * totalSizeH + pivotH) + globalSize];
			float z = data[(pivotV * totalSizeH + pivotH) + 2 * globalSize];

			for (int VIn = VOut; VIn < localSizeV + VOut; VIn++)
			{
				for (int HIn = HOut; HIn < localSizeH + HOut; HIn++)
				{
					float xd = data[VIn * totalSizeH + HIn];
					float yd = data[(VIn * totalSizeH + HIn) + globalSize];
					float zd = data[(VIn * totalSizeH + HIn) + 2 * globalSize];

					float difHeight = yd - y;

					float difX = xd - x;
					float difZ = zd - z;

					float norm = sqrt(difX * difX + difZ * difZ + difHeight * difHeight);

					float cosAlpha = difHeight / norm;

					if (difHeight > HMIN && difHeight < HMAX && cosAlpha > COSALPHA)
						img[VIn * totalSizeH + HIn] = 0;
				}
			}
		}
	}

}

cObstacleDetection::~cObstacleDetection()
{
}
