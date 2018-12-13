#include <stdio.h>
#include "model.h"

#define IDX(x, y, z, height, width) z * (height + y) * width + x);

void Model::init(int resolutionX, int resolutionY, int resolutionZ, float cellSize)
{
	if (resolutionX % 2 != 0 || resolutionY % 2 != 0 || resolutionZ % 2 != 0)
	{
		std::cout << "WARNING : resolution is not odd, center does not lie on grid" ;
	}
	this->resolutionX = resolutionX;
	this->resolutionY = resolutionY;
	this->resolutionZ = resolutionZ;
	
	this->tsdf = new float[resolutionX*resolutionY*resolutionZ];

	this->cellSize = cellSize;

}

// Constructor with default origin placement in camera centered coordinate system
// Origin is located at min (-resX/2, -resY/2 , -resZ/2)
Model::Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize)
{
	const float cornerX = - static_cast<float>(resolutionX) / 2 * cellSize;
	const float cornerY = - static_cast<float>(resolutionY) / 2 * cellSize;
	const float cornerZ = - static_cast<float>(resolutionZ) / 2 * cellSize;

	this->modelOrigin = Eigen::Vector3f(cornerX, cornerY, cornerZ);
}

Model::Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize, Eigen::Vector3f modelOrigin)
{
	init(resolutionX, resolutionY, resolutionZ, cellSize);

	//assert model in grid

	this->modelOrigin = modelOrigin;
}

Model::~Model()
{
	delete this->tsdf;
}

void Model::debugToLog()
{

	int x = resolutionX ;
	int y = resolutionY ;
	int z = resolutionZ ;

	std::cout << "Write contents of Model to cout";
	std::cout << "resolutionX" << x << "\n";
	std::cout << "resolutionX" << y << "\n";
	std::cout << "resolutionX" << z << "\n";

	std::cout << "tsdf of size" << x * y * z ;

	std::cout << "grid with cellSize : " << cellSize << "\n";
	std::cout << "total length in x : " << x * cellSize << "\n";
	std::cout << "total length in y : " << y * cellSize << "\n";
	std::cout << "total length in z : " << z * cellSize << "\n";

	std::cout << "model Origin at : " << modelOrigin << "\n";
}

