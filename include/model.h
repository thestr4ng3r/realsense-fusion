#ifndef _MODEL_H
#define _MODEL_H

#include <pcl/common/eigen.h>

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

		void debugToLog();

	private:
		void init(int resolutionX, int resolutionY, int resolutionZ, float cellSize);
};

#endif