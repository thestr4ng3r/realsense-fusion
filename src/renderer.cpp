
#include "window.h"
#include "renderer.h"
#include "gl_model.h"
#include "camera_transform.h"

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
"#version 450 core\n"
#include "glsl_common_grid.inl"
R"glsl(

uniform mat4 mvp_matrix;
uniform vec3 cam_pos;

layout(location = 0) in vec3 vertex_pos;

out vec3 world_pos;
out vec3 world_dir;

void main()
{
	world_pos = (vertex_pos * 0.5 + 0.5) * GridExtent() + grid_params.origin;
	world_dir = world_pos - cam_pos;
	gl_Position = mvp_matrix * vec4(world_pos, 1.0);
}
)glsl";

static const char *fragment_shader_code =
"#version 450 core\n"
#include "glsl_common_grid.inl"
R"glsl(
uniform sampler3D tsdf_tex;

in vec3 world_pos;
in vec3 world_dir;

out vec4 color_out;

float SDF(vec3 grid_pos)
{
	return texture(tsdf_tex, grid_pos).x;
}

vec3 Normal(vec3 world_pos, float epsilon)
{
	return normalize(vec3(
		SDF(WorldToGrid(world_pos + vec3(epsilon, 0.0, 0.0))) - SDF(WorldToGrid(world_pos - vec3(epsilon, 0.0, 0.0))),
		SDF(WorldToGrid(world_pos + vec3(0.0, epsilon, 0.0))) - SDF(WorldToGrid(world_pos - vec3(0.0, epsilon, 0.0))),
		SDF(WorldToGrid(world_pos + vec3(0.0, 0.0, epsilon))) - SDF(WorldToGrid(world_pos - vec3(0.0, 0.0, epsilon)))
	));
}

#define STEP_MIN 0.01

bool TraceRay(inout vec3 world_pos, vec3 world_dir)
{
	world_dir = normalize(world_dir);
	world_pos += world_dir * 0.00001;

	while(true)
	{
		vec3 grid_pos = WorldToGrid(world_pos);
		if(grid_pos.x < 0.0 || grid_pos.x > 1.0 || grid_pos.y < 0.0 || grid_pos.y > 1.0 || grid_pos.z < 0.0 || grid_pos.z > 1.0)
			return false;
		float world_dist = SDF(grid_pos);
		if(world_dist <= 0.0)
			return true;
		world_pos += world_dir * max(world_dist, STEP_MIN);
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
	vec3 world_pos_cur = world_pos;
	if(!TraceRay(world_pos_cur, world_dir))
		discard;
	vec3 normal = Normal(world_pos_cur, 0.001);

	float l = 0.1;
	l += 0.5 * Phong(normal, normalize(vec3(1.0, 1.0, 1.0)), 0.5, 64.0);

	color_out = vec4(vec3(l), 1.0);
}
)glsl";


Renderer::Renderer(Window *window)
{
	this->window = window;
	InitResources();
}

Renderer::~Renderer()
{
	glDeleteBuffers(1, &vbo);
	glDeleteBuffers(1, &ibo);
	glDeleteVertexArrays(1, &vao);
	glDeleteProgram(program);
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

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertex_data), vertex_data, GL_STATIC_DRAW);

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glEnableVertexAttribArray(ATTRIBUTE_VERTEX_POS);
	glVertexAttribPointer(ATTRIBUTE_VERTEX_POS, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

	glGenBuffers(1, &ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(index_data), index_data, GL_STATIC_DRAW);


	GLuint vert_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vert_shader, 1, &vertex_shader_code, nullptr);
	glCompileShader(vert_shader);

	GLuint frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(frag_shader, 1, &fragment_shader_code, nullptr);
	glCompileShader(frag_shader);

	program = glCreateProgram();
	glAttachShader(program, vert_shader);
	glAttachShader(program, frag_shader);
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

	glDeleteShader(vert_shader);
	glDeleteShader(frag_shader);

	mvp_matrix_uniform = glGetUniformLocation(program, "mvp_matrix");
	cam_pos_uniform = glGetUniformLocation(program, "cam_pos");
	tsdf_tex_uniform = glGetUniformLocation(program, "tsdf_tex");

	glUseProgram(program);
	glUniform1i(tsdf_tex_uniform, 0);
}

void Renderer::Render(GLModel *model, CameraTransform *camera_transform)
{
	int width, height;
	window->GetSize(&width, &height);

	Eigen::Vector3f cam_pos = camera_transform->GetTransform().translation();
	Eigen::Matrix4f modelview = camera_transform->GetModelView();
	Eigen::Matrix4f mvp_matrix = PerspectiveMatrix<float>(80.0f, (float)width / (float)height, 0.1f, 100.0f) * modelview.matrix();

	glUseProgram(program);
	glUniformMatrix4fv(mvp_matrix_uniform, 1, GL_FALSE, mvp_matrix.data());
	glUniform3fv(cam_pos_uniform, 1, cam_pos.data());
	glBindBufferBase(GL_UNIFORM_BUFFER, 0, model->GetParamsBuffer());
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_3D, model->GetTSDFTex());
	glBindVertexArray(vao);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_SHORT, nullptr);
}

