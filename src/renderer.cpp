
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "renderer.h"

#include <stdio.h>
#include <exception>


#define ATTRIBUTE_VERTEX_POS 0

static const char *vertex_shader_code =
	"#version 330 core\n"
	"\n"
	"layout(location = 0) in vec3 vertex_pos;\n"
	"\n"
	"void main()\n"
	"{\n"
	"	gl_Position = vec4(vertex_pos, 1.0);\n"
	"}\n";

static const char *fragment_shader_code =
	"#version 330 core\n"
	"\n"
 	"out vec4 color_out;"
 	"\n"
	"void main()\n"
	"{\n"
 	"	color_out = vec4(0.0, 0.0, 1.0, 1.0);"
	"}\n";

struct RendererInternal
{
	GLFWwindow *window = nullptr;
	GLuint vbo = 0;
	GLuint vao = 0;
	GLuint ibo = 0;
	GLuint program = 0;
};

Renderer::Renderer()
{
	if(!glfwInit())
		throw std::exception();

	internal = new RendererInternal();

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	internal->window = glfwCreateWindow(640, 480, "Renderer", nullptr, nullptr);

	if(!internal->window)
	{
		delete internal;
		glfwTerminate();
		throw std::exception();
	}

	glfwMakeContextCurrent(internal->window);
	glewInit();

	should_terminate = false;

	InitResources();
}

Renderer::~Renderer()
{
	glDeleteBuffers(1, &internal->vbo);
	glDeleteBuffers(1, &internal->ibo);
	glDeleteVertexArrays(1, &internal->vao);
	glDeleteProgram(internal->program);

	glfwDestroyWindow(internal->window);
	glfwTerminate();

	delete internal;
}

void Renderer::InitResources()
{
	static const float vertex_data[] = {
			-1.0f, -1.0f, -1.0f,
			1.0f, -1.0f, -1.0f,
			1.0f, 1.0f, -1.0f,
			-1.0f, 1.0f, -1.0f,
			-1.0f, -1.0f, 1.0f,
			1.0f, -1.0f, 1.0f,
			1.0f, 1.0f, 1.0f,
			-1.0f, 1.0f, 1.0f,
	};

	static const GLushort index_data[] = {
			3, 1, 0, 3, 2, 1, // back
			4, 5, 7, 5, 6, 7, // front
	};

	glGenBuffers(1, &internal->vbo);
	glBindBuffer(GL_ARRAY_BUFFER, internal->vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertex_data), vertex_data, GL_STATIC_DRAW);

	glGenVertexArrays(1, &internal->vao);
	glBindVertexArray(internal->vao);
	glEnableVertexAttribArray(ATTRIBUTE_VERTEX_POS);
	glVertexAttribPointer(ATTRIBUTE_VERTEX_POS, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

	glGenBuffers(1, &internal->ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, internal->ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(index_data), index_data, GL_STATIC_DRAW);


	GLuint vert_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vert_shader, 1, &vertex_shader_code, nullptr);
	glCompileShader(vert_shader);

	GLuint frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(frag_shader, 1, &fragment_shader_code, nullptr);
	glCompileShader(frag_shader);

	internal->program = glCreateProgram();
	glAttachShader(internal->program, vert_shader);
	glAttachShader(internal->program, frag_shader);
	glLinkProgram(internal->program);

	GLint result, log_len;
	glGetProgramiv(internal->program, GL_LINK_STATUS, &result);
	glGetProgramiv(internal->program, GL_INFO_LOG_LENGTH, &log_len);
	if(log_len > 0)
	{
		char *log = new char[log_len + 1];
		glGetProgramInfoLog(internal->program, log_len, nullptr, log);
		printf("%s\n", log);
		delete[] log;
	}

	glDeleteShader(vert_shader);
	glDeleteShader(frag_shader);
}

void Renderer::Update()
{
	int width, height;
	glfwGetFramebufferSize(internal->window, &width, &height);

	glViewport(0, 0, width, height);
	glClearColor(0.0, 1.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(internal->program);
	glBindVertexArray(internal->vao);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, internal->ibo);
	glDrawElements(GL_TRIANGLES, 12, GL_UNSIGNED_SHORT, nullptr);

	glfwSwapBuffers(internal->window);
	glfwPollEvents();

	should_terminate = static_cast<bool>(glfwWindowShouldClose(internal->window));
}

