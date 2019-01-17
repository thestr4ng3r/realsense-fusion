#ifndef _MODEL_H
#define _MODEL_H

#include <pcl/common/eigen.h>

#define DEBUG = 0;

class Model
{
	public:
		Model();
		Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize, float max_truncation, float min_truncation);
		Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize, float max_truncation, float min_truncation, Eigen::Vector3f modelOrigin);
		virtual ~Model();

		virtual void Reset() =0;

		int GetResolutionX()				{ return resolutionX; }
		int GetResolutionY()				{ return resolutionY; }
		int GetResolutionZ()				{ return resolutionZ; }
		float GetCellSize()					{ return cellSize; }
		Eigen::Vector3f GetModelOrigin()	{ return modelOrigin; }
		float GetMaxTruncation()			{ return max_truncation; }
		float GetMinTruncation()			{ return min_truncation; }

		Eigen::Vector3f GridToWorld(Eigen::Vector3f pos);
		Eigen::Vector3f WorldToGrid(Eigen::Vector3f pos);
		Eigen::Vector3i GridToTexel(Eigen::Vector3f pos);
		Eigen::Vector3f TexelToGrid(Eigen::Vector3i pos);

	private:
		void Init(int resolutionX, int resolutionY, int resolutionZ, float cellSize, float max_truncation, float min_truncation);

	protected:
		int resolutionX;
		int resolutionY;
		int resolutionZ;

		float cellSize;

		float max_truncation;
		float min_truncation;

		Eigen::Vector3f modelOrigin;
};

class CPUModel: public Model
{
	private:
		float *tsdf; //truncated signed distance function
		uint16_t *weigths;

		int IDX(int x, int y, int z) //3d index -> 1d index
		{
			return	(z * resolutionY * resolutionX) + (resolutionX * y) + x;
		}

		void Init();

	public:
		CPUModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize, float max_truncation, float min_truncation);
		CPUModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize, float max_truncation, float min_truncation, Eigen::Vector3f modelOrigin);
		~CPUModel() override;

		void Reset() override;

		float *GetData()					{ return tsdf; }
		uint16_t *GetWeights()				{ return weigths; }
		void GenerateSphere(float radius, Eigen::Vector3f center);

		void DebugToLog();
};

#endif