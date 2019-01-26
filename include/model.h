#ifndef _MODEL_H
#define _MODEL_H

#include <pcl/common/eigen.h>

#define DEBUG = 0;

class Model
{
	public:
		Model();
		Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize, float max_truncation, float min_truncation, bool colorsActive);
		Model(int resolutionX, int resolutionY, int resolutionZ, float cellSize, float max_truncation, float min_truncation, Eigen::Vector3f modelOrigin, bool colorsActive);
		virtual ~Model();

		virtual void Reset() =0;

		int GetResolutionX()				{ return resolutionX; }
		int GetResolutionY()				{ return resolutionY; }
		int GetResolutionZ()				{ return resolutionZ; }
		float GetCellSize()					{ return cellSize; }
		Eigen::Vector3f GetModelOrigin()	{ return modelOrigin; }
		float GetMaxTruncation()			{ return max_truncation; }
		float GetMinTruncation()			{ return min_truncation; }
		bool GetColorsActive()				{ return colorsActive; }

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

		bool colorsActive;
};

class CPUModel: public Model
{
	private:
		float *tsdf; //truncated signed distance function
		uint8_t *weights;
		uint8_t *color;

		int IDX(int x, int y, int z) //3d index -> 1d index
		{
			return	(z * resolutionY * resolutionX) + (resolutionX * y) + x;
		}

		void Init();

	public:
		CPUModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize, float max_truncation, float min_truncation, bool colorsActive);
		CPUModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize, float max_truncation, float min_truncation, Eigen::Vector3f modelOrigin, bool colorsActive);
		~CPUModel() override;

		void Reset() override;

		float *GetData()					{ return tsdf; }
		uint8_t *GetWeights()				{ return weights; }
		uint8_t *GetColor()					{ return color; }
		void GenerateSphere(float radius, Eigen::Vector3f center);

		void DebugToLog();
};

#endif