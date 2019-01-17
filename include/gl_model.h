
#ifndef _GL_MODEL_H
#define _GL_MODEL_H

#include "model.h"
#include "window.h"

class GLModel: public Model
{
	private:
		GLuint tsdf_tex;
		GLuint weight_tex;

		GLuint params_buffer;

		void Init();

	public:
		GLModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize, float max_truncation, float min_truncation);
		GLModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize, float max_truncation, float min_truncation, Eigen::Vector3f modelOrigin);
		~GLModel() override;

		void Reset() override;
		void CopyFrom(CPUModel *cpu_model);

		GLuint GetTSDFTex()			{ return tsdf_tex; }
		GLuint GetWeightTex()		{ return weight_tex; }
		GLuint GetParamsBuffer()	{ return params_buffer; }
};

#endif //_GL_MODEL_H
