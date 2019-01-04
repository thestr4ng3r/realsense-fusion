#ifndef _MODEL_H
#define _MODEL_H

#include <pcl/common/eigen.h>

#define DEBUG = 0;

class Model
{
	public:
		Model();
		Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize);
		Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize, Eigen::Vector3f modelOrigin);
		virtual ~Model();

		int GetResolutionX()				{ return resolutionX; }
		int GetResolutionY()				{ return resolutionY; }
		int GetResolutionZ()				{ return resolutionZ; }
		float GetCellSize()					{ return cellSize; }
		Eigen::Vector3f GetModelOrigin()	{ return modelOrigin; }

	private:
		void Init(int resolutionX, int resolutionY, int resolutionZ, float cellSize);

	protected:
		int resolutionX;
		int resolutionY;
		int resolutionZ;

		float cellSize;

		Eigen::Vector3f modelOrigin;

		void ApproximateModelPosition(int x, int y, int z, Eigen::Vector3f &pos);
};

class CPUModel: public Model
{
	private:
		float *tsdf; //truncated signed distance function
		float *weigths;

		int IDX(int x, int y, int z) //3d index -> 1d index
		{
			return	(z * resolutionY * resolutionX) + (resolutionX * y) + x;
		}

		void Init();

	public:
		CPUModel();
		CPUModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize);
		CPUModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize, Eigen::Vector3f modelOrigin);
		~CPUModel() override;

		float *GetData()					{ return tsdf; }
		float *GetWeights()					{ return weigths; }
		void GenerateSphere(float radius, Eigen::Vector3f center);
		void DebugToLog();
};

#endif