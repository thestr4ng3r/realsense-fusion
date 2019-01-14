
#ifndef _ICP_H
#define _ICP_H

#include "window.h"

class Frame;
class CameraTransform;

class ICP
{
	private:
		GLuint corr_program;

		GLuint corr_tex;

	public:
		ICP();
		virtual ~ICP();

		void SearchCorrespondences(Frame *frame, CameraTransform *cam_transform_old, CameraTransform *cam_transform_new);
};

#endif //_ICP_H
