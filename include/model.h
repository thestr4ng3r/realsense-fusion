#ifndef _MODEL_H
#define _MODEL_H

#include <pcl/common/eigen.h>

//#define DEBUG = 0;

class Model {

	public:
		Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize);
		Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize, Eigen::Vector3f modelOrigin);
		~Model();

		float* tsdf; //truncated signed distance function

		int resolutionX;
		int resolutionY;
		int resolutionZ;

		float cellSize;

		Eigen::Vector3f modelOrigin;

		void generateSphere(float radius, Eigen::Vector3f center);

		void debugToLog();

	private:
		void init(int resolutionX, int resolutionY, int resolutionZ, float cellSize);
		void approximateModelPosition(int x, int y, int z, Eigen::Vector3f& pos);
	
		int IDX(int x, int y, int z) //3d index -> 1d index
		{
			return	z * (resolutionY + y) * resolutionX + x;
		}
};

#endif