
#ifndef _PC_INTEGRATOR_H
#define _PC_INTEGRATOR_H

#include <GL/glew.h>
#include "frame.h"
#include "gl_model.h"
#include "input.h"

class GLModel;
class Window;

class PC_Integrator
{
	private:

		GLModel* glModel;

		GLuint computeHandle;
		GLuint intrinsic_center_uniform;
		GLuint intrinsic_focalLength_uniform;
		GLuint resolution_uniform;
		GLuint cellSize_uniform;
		GLuint depth_map_uniform;
		GLuint mvp_matrix_uniform;
		GLuint cam_pos_uniform;
		GLuint weight_tex_uniform;
		GLuint tsdf_tex_uniform;
		GLuint transposeInv_uniform;
		GLuint depth_scale_uniform;

		int resolutionX;
		int resolutionY;
		int resolutionZ;
		float cellSize;

		Input* input;

		GLuint genComputeProg();
		GLuint genTexture2D(int resolutionX, int resolutionY, float* data);

	public:
		PC_Integrator(GLModel* glModel, Input* input);
		~PC_Integrator();
		
		void integrate(Frame* frame);
};

#endif //_PC_INTEGRATOR_H