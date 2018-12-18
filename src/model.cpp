#include <stdio.h>
#include "model.h"
#include "implicit.h"




void Model::Init(int resolutionX, int resolutionY, int resolutionZ, float cellSize)
{
	if (resolutionX % 2 != 0 || resolutionY % 2 != 0 || resolutionZ % 2 != 0)
	{
		std::cout << "WARNING : resolution is not odd, center does not lie on grid" ;
	}
	this->resolutionX = resolutionX;
	this->resolutionY = resolutionY;
	this->resolutionZ = resolutionZ;
	
	this->tsdf = new float[resolutionX*resolutionY*resolutionZ];
	// init with -inf missing here

	this->cellSize = cellSize;

}

// Constructor with default origin placement in camera centered coordinate system
// Origin is located at min (-resX/2, -resY/2 , -resZ/2)
Model::Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize)
{
	Init(resolutionX, resolutionY, resolutionZ, cellSize);

	const float cornerX = - static_cast<float>(resolutionX) / 2.0f * cellSize;
	const float cornerY = - static_cast<float>(resolutionY) / 2.0f * cellSize;
	const float cornerZ = - static_cast<float>(resolutionZ) / 2.0f * cellSize;

	this->modelOrigin = Eigen::Vector3f(cornerX, cornerY, cornerZ);

}

Model::Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize, Eigen::Vector3f modelOrigin)
{
	Init(resolutionX, resolutionY, resolutionZ, cellSize);

	//assert model in grid

	this->modelOrigin = modelOrigin;
}

Model::~Model()
{
	delete this->tsdf;
}

void Model::GenerateSphere(float radius, Eigen::Vector3f center)
{
	_IMPLICIT_H::Sphere sphere(radius, center.x(), center.y(), center.z());
	//iterate over all sdf fields
	for (int z = 0; z < resolutionZ; z++)
	{
		for (int y = 0; y < resolutionY; y++)
		{
			for (int x = 0; x < resolutionX; x++)
			{
				Eigen::Vector3f pos;
				ApproximateModelPosition(x, y, z, pos);
//#ifdef DEBUG
//				std::cout << "grid value : " << x << " " << y << " " << z << "in Modelspace : " << pos.x() << " " << pos.y() << " " << pos.z() << "\n";
//#endif // DEBUG
				float eval = sphere.sdf(pos.x(), pos.y(), pos.z());
				int cellIndex = IDX(x, y, z);
				tsdf[cellIndex] = eval;
			}
		}
	}
}

//approximate center of sdf field by cell center and cell length
void Model::ApproximateModelPosition(int x, int y, int z, Eigen::Vector3f &pos)
{
	//scale in Model Space
	float x_field = (static_cast<float>(x) - resolutionX / 2.0f) * cellSize;
	float y_field = (static_cast<float>(y) - resolutionY / 2.0f) * cellSize;
	float z_field = (static_cast<float>(z) - resolutionZ / 2.0f) * cellSize;

	//shift into cell center
	x_field += cellSize / 2.0f;
	y_field += cellSize / 2.0f;
	z_field += cellSize / 2.0f;
	
	pos = Eigen::Vector3f(x_field, y_field, z_field);
}


void Model::DebugToLog()
{

	int x = resolutionX ;
	int y = resolutionY ;
	int z = resolutionZ ;

	std::cout << "Write contents of Model to cout \n";
	std::cout << "resolutionX :" << x << "\n";
	std::cout << "resolutionY :" << y << "\n";
	std::cout << "resolutionZ :" << z << "\n";

	std::cout << "tsdf of size :" << x * y * z ;

	std::cout << "grid with cellSize : " << cellSize << "\n";
	std::cout << "total length in x : " << x * cellSize << "\n";
	std::cout << "total length in y : " << y * cellSize << "\n";
	std::cout << "total length in z : " << z * cellSize << "\n";

	std::cout << "model Origin at : " << modelOrigin << "\n";

	std::cout << "Print contends of sdf : " << "\n";

	if (resolutionX * resolutionY < 16*16)
	{
		for (int z = 0; z < resolutionZ; z++)
		{
			std::cout << "Frame " << z << "\n";

			for (int y = 0; y < resolutionY; y++)
			{
				for (int x = 0; x < resolutionX; x++)
				{
					std::cout << tsdf[IDX(x,y,z)] << " ";
				}
				std::cout << "\n";
			}
		}
	}
}

