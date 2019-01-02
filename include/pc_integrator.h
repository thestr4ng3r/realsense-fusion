
#ifndef _PC_INTEGRATOR_H
#define _PC_INTEGRATOR_H

#include <GL/glew.h>
#include "frame.h"
#include "gl_model.h"

class GLModel;
class Window;

class PC_Integrator
{
	private:

		GLModel glModel;
		CPUModel cpumodel;

		GLuint computeHandle;

		GLuint depth_map_uniform;
		GLuint mvp_matrix_uniform;
		GLuint cam_pos_uniform;
		GLuint tsdf_tex_uniform;
		GLuint transposeInv_uniform;

		int resolutionX;
		int resolutionY;
		int resolutionZ;
		float cellSize;

		GLuint genComputeProg();
		GLuint genTexture2D(int resolutionX, int resolutionY, float* data);

	public:
		PC_Integrator(CPUModel &cpuModel);
		~PC_Integrator();
		
		void integrate(Frame &frame);
};

#endif //_PC_INTEGRATOR_H