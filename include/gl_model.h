
#ifndef _GL_MODEL_H
#define _GL_MODEL_H

#include "model.h"
#include "window.h"

class GLModel: public Model
{
	private:
		GLuint tsdf_tex;
		GLuint weight_tex;

		void Init();

	public:
		GLModel();
		GLModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize);
		GLModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize, Eigen::Vector3f modelOrigin);
		~GLModel() override;

		void CopyFrom(CPUModel *cpu_model);

		GLuint GetTSDFTex()	{ return tsdf_tex; }
		GLuint GetWeightTex() { return weight_tex; }
};

#endif //_GL_MODEL_H
