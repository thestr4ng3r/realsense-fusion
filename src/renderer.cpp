
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "renderer.h"

#include <stdio.h>
#include <exception>

#include <Eigen/Core>
#include <Eigen/Geometry>


template<class T>
Eigen::Matrix<T, 4, 4> PerspectiveMatrix(T fovy, T aspect, T near_clip, T far_clip)
{
	T fovy_rad = fovy * M_PI / 180.0;
	T f = 1.0 / std::tan(fovy_rad * 0.5);
	T nmfi = 1.0 / (near_clip - far_clip);
	Eigen::Matrix<T, 4, 4> r;
	r << (f / aspect), 0.0, 0.0, 0.0,
		0.0, f, 0.0, 0.0,
		0.0, 0.0, (far_clip + near_clip) * nmfi, 2.0 * far_clip * near_clip * nmfi,
		0.0, 0.0, -1.0, 0.0;
	return r;
}



#define ATTRIBUTE_VERTEX_POS 0

static const char *vertex_shader_code =
R"glsl(
#version 330 core

uniform mat4 mvp_matrix;
uniform vec3 cam_pos;

layout(location = 0) in vec3 vertex_pos;

out vec3 grid_pos;
out vec3 grid_dir;

vec3 WorldToGrid(vec3 pos)
{
	return pos * 0.5 + 0.5;
}

void main()
{
	grid_pos = WorldToGrid(vertex_pos);
	vec3 dir_from_cam = vertex_pos - cam_pos;
	grid_dir = WorldToGrid(vertex_pos + normalize(dir_from_cam)) - grid_pos;
	gl_Position = mvp_matrix * vec4(vertex_pos, 1.0);
}
)glsl";

static const char *fragment_shader_code =
R"glsl(
#version 330 core

in vec3 grid_pos;
in vec3 grid_dir;

out vec4 color_out;

float SDF(vec3 pos)
{
	vec3 dir = pos - vec3(0.5, 0.5, 0.5);
	return length(dir) - 0.5;
}

vec3 Normal(vec3 pos, float epsilon)
{
	return normalize(vec3(
		SDF(pos + vec3(epsilon, 0.0, 0.0)) - SDF(pos - vec3(epsilon, 0.0, 0.0)),
		SDF(pos + vec3(0.0, epsilon, 0.0)) - SDF(pos - vec3(0.0, epsilon, 0.0)),
		SDF(pos + vec3(0.0, 0.0, epsilon)) - SDF(pos - vec3(0.0, 0.0, epsilon))
	));
}

bool TraceRay(inout vec3 pos, vec3 dir)
{
	dir = normalize(dir);
	pos += dir * 0.00001;

	while(true)
	{
		if(pos.x < 0.0 || pos.x > 1.0 || pos.y < 0.0 || pos.y > 1.0 || pos.z < 0.0 || pos.z > 1.0)
			return false;
		float dist = SDF(pos);
		if(dist <= 0.0)
			return true;
		pos += dir * dist;
	}
}

float Lambert(vec3 normal, vec3 light_dir)
{
	return max(0.0, dot(normal, light_dir));
}

float Phong(vec3 normal, vec3 light_dir, float specular, float exponent)
{
	float lambert = Lambert(normal, light_dir);
	float spec = pow(lambert, exponent) * specular;
	return lambert + spec;
}

void main()
{
	vec3 pos = grid_pos;
	if(!TraceRay(pos, grid_dir))
		discard;
	vec3 normal = Normal(pos, 0.001);

	float l = 0.1;
	l += 0.5 * Phong(normal, normalize(vec3(1.0, 1.0, 1.0)), 0.5, 64.0);

	color_out = vec4(vec3(l), 1.0);
}
)glsl";

struct RendererInternal
{
	GLFWwindow *window = nullptr;

	GLuint vbo = 0;
	GLuint vao = 0;
	GLuint ibo = 0;
	GLuint program = 0;

	GLint mvp_matrix_uniform = -1;
	GLint cam_pos_uniform = -1;
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
			7, 0, 4, 7, 3, 0, // left
			6, 5, 1, 6, 1, 2, // right
			7, 6, 2, 7, 2, 3, // top
			4, 1, 5, 4, 0, 1, // bottom
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

	internal->mvp_matrix_uniform = glGetUniformLocation(internal->program, "mvp_matrix");
	internal->cam_pos_uniform = glGetUniformLocation(internal->program, "cam_pos");
}

void Renderer::Update()
{
	int width, height;
	glfwGetFramebufferSize(internal->window, &width, &height);

	glViewport(0, 0, width, height);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);

	Eigen::Vector3f cam_pos(0.0f, 0.0f, 3.0f);

	Eigen::Affine3f modelview = Eigen::Affine3f::Identity();
	modelview.translate(-cam_pos);
	//modelview.rotate(Eigen::AngleAxisf(0.25f * M_PI, Eigen::Vector3f::UnitX()));

	Eigen::Matrix4f mvp_matrix;
	mvp_matrix = PerspectiveMatrix<float>(80.0f, (float)width / (float)height, 0.1f, 100.0f) * modelview.matrix();

	Eigen::Matrix4f mvp_matrix_inv = mvp_matrix.inverse();

	glUseProgram(internal->program);
	glUniformMatrix4fv(internal->mvp_matrix_uniform, 1, GL_FALSE, mvp_matrix.data());
	glUniform3fv(internal->cam_pos_uniform, 1, cam_pos.data());
	glBindVertexArray(internal->vao);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, internal->ibo);
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_SHORT, nullptr);

	glfwSwapBuffers(internal->window);
	glfwPollEvents();

	should_terminate = static_cast<bool>(glfwWindowShouldClose(internal->window));
}

