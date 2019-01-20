
#ifndef _PC_INTEGRATOR_H
#define _PC_INTEGRATOR_H

#include <GL/glew.h>
#include "frame.h"
#include "gl_model.h"
#include "input.h"

class GLModel;
class Window;
class CameraTransform;

class PC_Integrator
{
	private:

		GLModel* glModel;

		GLuint computeHandle;
		GLint depth_map_uniform;
		GLint cam_modelview_uniform;
		GLint cam_pos_uniform;
		GLint cam_dir_uniform;
		GLint weight_tex_uniform;
		GLint tsdf_tex_uniform;
		GLint depth_scale_uniform;
		GLint max_truncation_uniform;
		GLint min_truncation_uniform;
		GLint max_weight_uniform;
		GLint activateColors_uniform;

		int resolutionX;
		int resolutionY;
		int resolutionZ;
		float cellSize;

		GLuint genComputeProg();
		GLuint genTexture2D(int resolutionX, int resolutionY, float* data);

	public:
		PC_Integrator(GLModel* glModel);
		~PC_Integrator();
		
		void integrate(Frame* frame, CameraTransform *camera_transform);
};

#endif //_PC_INTEGRATOR_H