
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
		GLuint computeHandle;
		GLuint tsdfHandle;
		GLuint depthMapHandle;

		int resolutionX;
		int resolutionY;
		int resolutionZ;

		GLuint genComputeProg();
		GLuint genTexture(int resolution);
		//Gluint genDepthMapTexture(Input);  ToDo

	public:
		PC_Integrator(GLModel glmodel);
		~PC_Integrator();
		
		void integrate_Frame(Frame frame);
};

#endif //_PC_INTEGRATOR_H