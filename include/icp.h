
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
		GLint corr_image_res_uniform;

		GLuint residuals_buffer;
		unsigned int residuals_count;

		GLuint reduce_program;
		GLint reduce_residuals_count_uniform;

		GLuint matrix_buffer;

		float distance_threshold;
		float angle_threshold;

#ifdef ICP_DEBUG_TEX
		GLuint debug_tex;
		int debug_tex_width;
		int debug_tex_height;
#endif

	public:
		ICP();
		virtual ~ICP();

		void SearchCorrespondences(Frame *frame, Renderer *renderer, const CameraTransform &cam_transform_current);
		void SolveMatrix(CameraTransform *cam_transform);

		float GetDistanceThreshold()		{ return distance_threshold; }
		float GetAngleThreshold()			{ return angle_threshold; }

		void SetDistanceThreshold(float v)	{ distance_threshold = v; }
		void SetAngleThreshold(float v)		{ angle_threshold = v; }

#ifdef ICP_DEBUG_TEX
		GLuint GetDebugTex()				{ return debug_tex; }
		int GetDebugTexWidth()				{ return debug_tex_width; }
		int GetDebugTexHeight()				{ return debug_tex_height; }
#endif
};

#endif //_ICP_H
