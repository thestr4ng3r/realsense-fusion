#include "pc_integrator.h"
#include "GL/glew.h"


PC_Integrator::PC_Integrator(GLModel glmodel)
{
	this->resolutionX = glmodel.GetResolutionX();
	this->resolutionY = glmodel.GetResolutionY();
	this->resolutionZ = glmodel.GetResolutionZ();

	//maybe assert that resolution is even and in an dimensions identic
	// TODO : // Access Depth Image from Camera Input class
	int depthRes = resolutionX;

	this->tsdfHandle = glmodel.GetTSDFTex();
	this->depthMapHandle = genTexture(depthRes);  

	this->computeHandle = genComputeProg();
}

GLuint PC_Integrator::genComputeProg()
{
	GLuint progHandle = glCreateProgram();
	GLuint cs = glCreateShader(GL_COMPUTE_SHADER);

	const char *csSrc[] = {
		"#version 430\n",
		"uniform float res;\
		uniform image2D depth;\
		uniform image3D tsdf;\
		layout (local_size_x = res, local_size_y = res) in;\
		void main() {}\
			ivec2 xy = ivec(gl_globalInvocationID.xy)\
		todo...\
		}"		
	};

	glShaderSource(cs, 2, csSrc, NULL);
	glCompileShader(cs);
	int rvalue;
	glGetShaderiv(cs, GL_COMPILE_STATUS, &rvalue);
	if (!rvalue)
	{
		fprintf(stderr, "Error in compiling the compute shader\n");
		GLchar log[10240];
		GLsizei length;
		glGetShaderInfoLog(cs, 10239, &length, log);
		fprintf(stderr, "Compiler log:\n%s\n", log);
		exit(40);
	}
	glAttachShader(progHandle, cs);

	glLinkProgram(progHandle);
	glGetProgramiv(progHandle, GL_LINK_STATUS, &rvalue);
	if (!rvalue) {
		fprintf(stderr, "Error in linking compute shader program\n");
		GLchar log[10240];
		GLsizei length;
		glGetProgramInfoLog(progHandle, 10239, &length, log);
		fprintf(stderr, "Linker log:\n%s\n", log);
		exit(41);
	}
	glUseProgram(progHandle);

	glUniform1i(glGetUniformLocation(progHandle, "tsdf"), 0);

	return progHandle;
}

GLuint genTexture2D(int resolution)
{
	GLuint texHandle;
	glGenTextures(1, &texHandle);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, texHandle);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	//Bind texture as image to allow read write operations
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, resolution, resolution, 0, GL_RED, GL_FLOAT, NULL); //add here input
	glBindImageTexture(0, texHandle, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32F);

	return texHandle;
}