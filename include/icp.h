
#ifndef _ICP_H
#define _ICP_H

#include "window.h"

class Frame;
class CameraTransform;
class Renderer;

//#define ICP_DEBUG_TEX

class ICP
{
	private:
		GLuint corr_program;
		GLint corr_distance_sq_threshold_uniform;
		GLint corr_angle_cos_threshold_uniform;
		GLint corr_modelview_prev_uniform;
		GLint corr_projection_prev_uniform;
		GLint corr_transform_current_uniform;
		GLint corr_image_width_uniform;

		GLuint residuals_buffer;
		size_t residuals_buffer_size;

#ifdef ICP_DEBUG_TEX
		GLuint debug_tex;
		int debug_tex_width;
		int debug_tex_height;
#endif

	public:
		ICP();
		virtual ~ICP();

		void SearchCorrespondences(Frame *frame, Renderer *renderer, const CameraTransform &cam_transform_old, CameraTransform *cam_transform_new);
};

#endif //_ICP_H
