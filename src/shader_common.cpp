
#include "shader_common.h"

#include <stdio.h>

GLuint CreateComputeShader(const char *source)
{
	GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
	glShaderSource(shader, 1, &source, nullptr);
	glCompileShader(shader);

	GLuint program = glCreateProgram();
	glAttachShader(program, shader);
	glLinkProgram(program);

	GLint result, log_len;
	glGetProgramiv(program, GL_LINK_STATUS, &result);
	glGetProgramiv(program, GL_INFO_LOG_LENGTH, &log_len);
	if(log_len > 0)
	{
		char *log = new char[log_len + 1];
		glGetProgramInfoLog(program, log_len, nullptr, log);
		printf("%s\n", log);
		delete[] log;
	}

	glDeleteShader(shader);

	return program;
}