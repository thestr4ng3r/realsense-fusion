
#include "window.h"
#include "renderer.h"
#include "gl_model.h"
#include "camera_transform.h"
#include "frame.h"

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

Eigen::Matrix4f CameraIntrinsicsMatrix(Eigen::Vector2f f, Eigen::Vector2f center, const Eigen::Vector2f &res, float near_clip, float far_clip)
{
	f = f.cwiseQuotient(res) * 2.0f;
	center = center.cwiseQuotient(res) * 2.0f - Eigen::Vector2f(1.0f, 1.0f);
	float nmfi = 1.0f / (near_clip - far_clip);
	Eigen::Matrix4f r;
	r <<	f.x(),	0.0,	center.x(),						0.0,
			0.0,	f.y(),	center.y(),						0.0,
			0.0,	0.0,	(far_clip + near_clip) * nmfi,	2.0 * far_clip * near_clip * nmfi,
			0.0,	0.0,	-1.0,							0.0;
	return r;
}




#define ATTRIBUTE_VERTEX_POS 0

static const char *vertex_shader_code =
"#version 450 core\n"
#include "glsl_common_grid.inl"
R"glsl(

uniform mat4 mvp_matrix;
uniform vec3 cam_pos;
uniform int activate_colors;


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
layout(binding = 0) uniform sampler3D tsdf_tex;
layout(binding = 1) uniform sampler3D color_grid_tex;

uniform mat4 modelview_matrix;

uniform vec3 cam_pos;

uniform int activate_colors;

in vec3 world_pos;
in vec3 world_dir;

layout(location = 0) out vec4 color_out;
layout(location = 1) out vec3 vertex_out;
layout(location = 2) out vec3 normal_out;

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
	//world_pos += world_dir * 0.00001;

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

vec4 PhongColor(vec4 c ,vec3 normal, vec3 light_dir, float specular, float exponent)
{
	float lambert = Lambert(normal, light_dir) ;
	float spec = pow(lambert, exponent) * specular ;
	return vec4(lambert + spec) * c;
}

float RayBoxIntersection(vec3 origin, vec3 dir, in vec3 box_min, in vec3 box_max)
{
	// see Williams, Amy, et al. "An efficient and robust ray-box intersection algorithm."

    vec3 dir_inv = vec3(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);

    float tmin;
    if(dir_inv.x >= 0.0)
        tmin = (box_min.x - origin.x) * dir_inv.x;
    else
        tmin = (box_max.x - origin.x) * dir_inv.x;

    if(dir_inv.y >= 0.0)
        tmin = max(tmin, (box_min.y - origin.y) * dir_inv.y);
    else
        tmin = max(tmin, (box_max.y - origin.y) * dir_inv.y);

    if(dir_inv.z >= 0.0)
        tmin = max(tmin, (box_min.z - origin.z) * dir_inv.z);
    else
        tmin = max(tmin, (box_max.z - origin.z) * dir_inv.z);

	return tmin;
}

void main()
{
	float dist = RayBoxIntersection(cam_pos, world_dir, GridBoxWorldMin(), GridBoxWorldMax());
	vec3 world_pos_cur = cam_pos + world_dir * max(dist, 0.0001);

	if(!TraceRay(world_pos_cur, world_dir))
		discard;
	vec3 normal = Normal(world_pos_cur, 0.001);

	vec4 l = vec4(0.0, 0.0, 0.0, 0.0);
	l += 0.5 * Phong(normal,normalize(vec3(1.0, 1.0, 1.0)), 0.5, 64.0);
	
	if(activate_colors!=0)
	{
		vec4 color = texture(color_grid_tex, WorldToGrid(world_pos_cur), 0);
		l = PhongColor(color, normal,normalize(vec3(1.0, 1.0, 1.0)), 0.5, 64.0);
	}
	//vec4 screen_coord = mvp_matrix * vec4(world_pos_cur, 1.0);
	//gl_FragDepth = screen_coord.z / screen_coord.w;
	color_out = l;
	vertex_out = world_pos_cur;
	normal_out = normal;
}
)glsl";



static const char *box_vertex_shader_code =
"#version 450 core\n"
#include "glsl_common_grid.inl"
R"glsl(

uniform mat4 mvp_matrix;

layout(location = 0) in vec3 vertex_pos;

out vec3 grid_pos;

void main()
{
	grid_pos = vertex_pos * 0.5 + 0.5;
	vec3 world_pos = (grid_pos) * GridExtent() + grid_params.origin;
	gl_Position = mvp_matrix * vec4(world_pos, 1.0);
}
)glsl";

static const char *box_fragment_shader_code =
"#version 450 core\n"
#include "glsl_common_grid.inl"
R"glsl(

in vec3 grid_pos;

out vec4 color_out;

void main()
{
	vec3 m = grid_pos * 8.0;
	m -= floor(m);
	vec3 color;
	if(m.x > 0.5 ^^ m.y > 0.5 ^^ m.z > 0.5)
		color = vec3(0.1, 0.1, 0.2);
	else
		color = vec3(0.1, 0.2, 0.1);
	color_out = vec4(color, 1.0);
}
)glsl";


Renderer::Renderer(Window *window, bool renderColor)
{
	this->window = window;
	this->renderColor = renderColor;
	InitResources();
}

Renderer::~Renderer()
{
	glDeleteBuffers(1, &vbo);
	glDeleteBuffers(1, &ibo);
	glDeleteVertexArrays(1, &vao);
	glDeleteProgram(program);
	glDeleteFramebuffers(1, &fbo);
	glDeleteTextures(1, &color_tex);
	glDeleteTextures(1, &depth_tex);
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
	glObjectLabel(GL_BUFFER, vbo, -1, "Renderer::vbo");

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glEnableVertexAttribArray(ATTRIBUTE_VERTEX_POS);
	glVertexAttribPointer(ATTRIBUTE_VERTEX_POS, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
	glObjectLabel(GL_VERTEX_ARRAY, vao, -1, "Renderer::vao");

	glGenBuffers(1, &ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(index_data), index_data, GL_STATIC_DRAW);
	glObjectLabel(GL_BUFFER, ibo, -1, "Renderer::ibo");


	{
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

		int rvalue;
		glGetShaderiv(frag_shader, GL_COMPILE_STATUS, &rvalue);
		if (!rvalue)
		{
			fprintf(stderr, "Error in compiling the compute shader\n");
			GLchar log[10240];
			GLsizei length;
			glGetShaderInfoLog(frag_shader, 10239, &length, log);
			fprintf(stderr, "Compiler log:\n%s\n", log);
			exit(40);
		}

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
	}

	glObjectLabel(GL_PROGRAM, program, -1, "Renderer::program");

	mvp_matrix_uniform = glGetUniformLocation(program, "mvp_matrix");
	modelview_matrix_uniform = glGetUniformLocation(program, "modelview_matrix");
	cam_pos_uniform = glGetUniformLocation(program, "cam_pos");
	tsdf_tex_uniform = glGetUniformLocation(program, "tsdf_tex");
	activate_colors_uniform = glGetUniformLocation(program, "activate_colors");
	if(renderColor)
		color_grid_tex_uniform = glGetUniformLocation(program, "color_grid_tex");

	glUseProgram(program);
	glUniform1i(tsdf_tex_uniform, 0);
	if (renderColor)
		glUniform1i(color_grid_tex_uniform, 1);

	{
		GLuint vert_shader = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(vert_shader, 1, &box_vertex_shader_code, nullptr);
		glCompileShader(vert_shader);

		GLuint frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(frag_shader, 1, &box_fragment_shader_code, nullptr);
		glCompileShader(frag_shader);

		box_program = glCreateProgram();
		glAttachShader(box_program, vert_shader);
		glAttachShader(box_program, frag_shader);
		glLinkProgram(box_program);

		GLint result, log_len;
		glGetProgramiv(box_program, GL_LINK_STATUS, &result);
		glGetProgramiv(box_program, GL_INFO_LOG_LENGTH, &log_len);
		if(log_len > 0)
		{
			char *log = new char[log_len + 1];
			glGetProgramInfoLog(box_program, log_len, nullptr, log);
			printf("%s\n", log);
			delete[] log;
		}

		glDeleteShader(vert_shader);
		glDeleteShader(frag_shader);
	}

	glObjectLabel(GL_PROGRAM, box_program, -1, "Renderer::box_program");

	box_mvp_matrix_uniform = glGetUniformLocation(box_program, "mvp_matrix");

	fbo_width = fbo_height = -1;
	glCreateFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	glGenTextures(1, &color_tex);
	glBindTexture(GL_TEXTURE_2D, color_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color_tex, 0);
	glObjectLabel(GL_TEXTURE, color_tex, -1, "Renderer::color_tex");

	glGenTextures(1, &vertex_tex);
	glBindTexture(GL_TEXTURE_2D, vertex_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, vertex_tex, 0);
	glObjectLabel(GL_TEXTURE, vertex_tex, -1, "Renderer::vertex_tex");

	glGenTextures(1, &normal_tex);
	glBindTexture(GL_TEXTURE_2D, normal_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, normal_tex, 0);
	glObjectLabel(GL_TEXTURE, normal_tex, -1, "Renderer::normal_tex");

	glGenTextures(1, &depth_tex);
	glBindTexture(GL_TEXTURE_2D, depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_tex, 0);
	glObjectLabel(GL_TEXTURE, depth_tex, -1, "Renderer::depth_tex");

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Renderer::Render(GLModel *model, Frame *frame, CameraTransform *camera_transform)
{
	int width = frame->GetDepthWidth();
	int height = frame->GetDepthHeight();

	renderColor = model->GetColorsActive();

	if(width != fbo_width || height != fbo_height)
	{
		fbo_width = width;
		fbo_height = height;
		glBindTexture(GL_TEXTURE_2D, color_tex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
		glBindTexture(GL_TEXTURE_2D, vertex_tex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
		glBindTexture(GL_TEXTURE_2D, normal_tex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
		glBindTexture(GL_TEXTURE_2D, depth_tex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
	}

	float vertex_clear[] = { INFINITY, INFINITY, INFINITY, INFINITY };
	glClearTexImage(vertex_tex, 0, GL_RGBA, GL_FLOAT, vertex_clear);

	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	GLenum fbo_state = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if(fbo_state != GL_FRAMEBUFFER_COMPLETE)
	{
		const char *state_name;
		switch(fbo_state)
		{
			case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
				state_name = "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
				state_name = "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
				state_name = "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
				state_name = "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER";
				break;
			case GL_FRAMEBUFFER_UNSUPPORTED:
				state_name = "GL_FRAMEBUFFER_UNSUPPORTED";
				break;
			default:
				state_name = "?";
				break;
		}
		printf("fbo status: %s\n", state_name);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		return;
	}
	glViewport(0, 0, width, height);
	glDrawBuffer(GL_COLOR_ATTACHMENT0);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	Eigen::Vector3f cam_pos = camera_transform->GetTransform().translation();
	modelview_matrix = camera_transform->GetModelView();
	projection_matrix = CameraIntrinsicsMatrix(
			frame->GetIntrinsicsFocalLength(),
			frame->GetIntrinsicsCenter(),
			Eigen::Vector2f(frame->GetDepthWidth(), frame->GetDepthHeight()),
			0.1f, 100.0f);
	//projection = PerspectiveMatrix<float>(80.0f, (float)width / (float)height, 0.1f, 100.0f);
	//std::cout << "projection:\n" << projection << std::endl;
	Eigen::Matrix4f mvp_matrix = projection_matrix * modelview_matrix;

	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);

	glBindVertexArray(vao);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
	glBindBufferBase(GL_UNIFORM_BUFFER, 0, model->GetParamsBuffer());
	glBindBufferBase(GL_UNIFORM_BUFFER, 1, frame->GetCameraIntrinsicsBuffer());

	glUseProgram(box_program);
	glUniformMatrix4fv(box_mvp_matrix_uniform, 1, GL_FALSE, mvp_matrix.data());
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_SHORT, nullptr);

	GLenum draw_buffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
	glDrawBuffers(3, draw_buffers);

	glUseProgram(program);
	glUniformMatrix4fv(mvp_matrix_uniform, 1, GL_FALSE, mvp_matrix.data());
	glUniformMatrix4fv(modelview_matrix_uniform, 1, GL_FALSE, modelview_matrix.data());
	glUniform3fv(cam_pos_uniform, 1, cam_pos.data());
	glUniform1i(activate_colors_uniform, renderColor);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_3D, model->GetTSDFTex());
	if (renderColor)
	{
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_3D, model->GetColorTex());
	}
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_SHORT, nullptr);

	glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	glReadBuffer(GL_COLOR_ATTACHMENT0);
	glDrawBuffer(GL_BACK);
	int window_width, window_height;
	window->GetSize(&window_width, &window_height);

	float frame_aspect = (float)width / (float)height;
	float window_aspect = (float)window_width / (float)window_height;
	int dst_width, dst_height;
	if(frame_aspect > window_aspect)
	{
		dst_width = window_width;
		dst_height = (int)((float)dst_width / frame_aspect);
	}
	else
	{
		dst_height = window_height;
		dst_width = (int)((float)dst_height * frame_aspect);
	}
	int dst_x = (window_width - dst_width) / 2;
	int dst_y = (window_height - dst_height) / 2;

	glBlitFramebuffer(0, 0, width, height, dst_x, dst_y, dst_x + dst_width, dst_y + dst_height, GL_COLOR_BUFFER_BIT, GL_LINEAR);
	glViewport(0, 0, window_width, window_height);
}

