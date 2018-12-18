#ifndef _MODEL_H
#define _MODEL_H

#include <pcl/common/eigen.h>

#define DEBUG = 0;

class Model
{
	public:
		Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize);
		Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize, Eigen::Vector3f modelOrigin);
		~Model();

		float *GetData()					{ return tsdf; }
		int GetResolutionX()				{ return resolutionX; }
		int GetResolutionY()				{ return resolutionY; }
		int GetResolutionZ()				{ return resolutionZ; }
		float GetCellSize()					{ return cellSize; }
		Eigen::Vector3f GetModelOrigin()	{ return modelOrigin; }
		
		void GenerateSphere(float radius, Eigen::Vector3f center);

		void DebugToLog();

	private:
		float* tsdf; //truncated signed distance function

		int resolutionX;
		int resolutionY;
		int resolutionZ;

		float cellSize;

		Eigen::Vector3f modelOrigin;
		
		void Init(int resolutionX, int resolutionY, int resolutionZ, float cellSize);
		void ApproximateModelPosition(int x, int y, int z, Eigen::Vector3f &pos);
	
		int IDX(int x, int y, int z) //3d index -> 1d index
		{
			return	(z * resolutionY * resolutionX) + (resolutionX * y) + x;
		}
};

#endif