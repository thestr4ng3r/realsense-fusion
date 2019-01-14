
#ifndef _ICP_H
#define _ICP_H

#include "window.h"

class Frame;
class CameraTransform;

class ICP
{
	private:
		GLuint corr_program;

		GLuint residuals_buffer;
		size_t residuals_buffer_size;

	public:
		ICP();
		virtual ~ICP();

		void SearchCorrespondences(Frame *frame, const CameraTransform &cam_transform_old, CameraTransform *cam_transform_new);
};

#endif //_ICP_H
