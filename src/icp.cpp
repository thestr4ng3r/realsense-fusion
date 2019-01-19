
#include <icp.h>

#include "icp.h"
#include "frame.h"
#include "camera_transform.h"
#include "shader_common.h"
#include "renderer.h"

#define RESIDUAL_COMPONENTS 7
#define MATRIX_COLUMNS RESIDUAL_COMPONENTS
#define MATRIX_ROWS (RESIDUAL_COMPONENTS-1)
#define ICP_DISTANCE_THRESHOLD 0.3f
#define ICP_ANGLE_COS_TRESHOLD 0.0f

#define STRHELPER(x) #x
#define TOSTR(x) STRHELPER(x)

static const char *corr_shader_code =
"#version 450 core\n"
#ifdef ICP_DEBUG_TEX
"#define ICP_DEBUG_TEX\n"
#endif
"#line " TOSTR(__LINE__) "\n" R"glsl(

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

uniform mat4 modelview_prev;
uniform mat4 projection_prev;

layout(binding = 0) uniform sampler2D vertex_tex_prev;
layout(binding = 1) uniform sampler2D normal_tex_prev;

uniform mat4 transform_current;

layout(binding = 2) uniform sampler2D vertex_tex_current;
layout(binding = 3) uniform sampler2D normal_tex_current;

uniform float distance_sq_threshold;
uniform float angle_cos_threshold;

uniform int image_width;

layout(std430, binding = 0) buffer residuals_out
{
	float residuals[];
};

#ifdef ICP_DEBUG_TEX
layout(rgba32f, binding = 0) uniform image2D debug_out;
#endif

ivec2 coord;

void StoreResidual(vec3 a, vec3 b, float c)
{
	uint base = 7 * uint(coord.y * image_width + coord.x);
	residuals[base+0] = a.x;
	residuals[base+1] = a.y;
	residuals[base+2] = a.z;
	residuals[base+3] = b.x;
	residuals[base+4] = b.y;
	residuals[base+5] = b.z;
	residuals[base+6] = c;
}

void StoreNopResidual()
{
	StoreResidual(vec3(0.0), vec3(0.0), 0.0);
}

void main()
{
	// see also https://github.com/chrdiller/KinectFusionLib/blob/master/src/cuda/pose_estimation.cu
	// but we're not building a pyramid of doom

	coord = ivec2(gl_GlobalInvocationID.xy);
	vec3 vertex_current_camera = texelFetch(vertex_tex_current, coord, 0).xyz;
	if(isinf(vertex_current_camera.x))
	{
		StoreNopResidual();
		return;
	}

	vec3 vertex_current_world = (transform_current * vec4(vertex_current_camera, 1.0)).xyz;
	vec3 vertex_current_camera_prev = (modelview_prev * vec4(vertex_current_world, 1.0)).xyz;
	vec4 vertex_current_image_prev_p = projection_prev * vec4(vertex_current_camera_prev, 1.0);
	vec2 vertex_current_image_prev = (vertex_current_image_prev_p.xy / vertex_current_image_prev_p.w) * 0.5 + 0.5;
	if(vertex_current_image_prev.x < 0.0 || vertex_current_image_prev.y < 0.0
		|| vertex_current_image_prev.x > 1.0 || vertex_current_image_prev.y > 1.0
		|| vertex_current_camera_prev.z > 0.0)
	{
		StoreNopResidual();
		return;
	}

	vec3 vertex_prev_world = texture(vertex_tex_prev, vertex_current_image_prev).xyz;
	if(isinf(vertex_prev_world.x))
	{
		StoreNopResidual();
		return;
	}


	vec3 dir_world = vertex_prev_world - vertex_current_world;
	float dist_sq = dot(dir_world, dir_world);
	if(dist_sq > distance_sq_threshold)
	{
		StoreNopResidual();
		return;
	}

	vec3 normal_prev_world = texture(normal_tex_prev, vertex_current_image_prev).xyz;
	vec3 normal_current_camera = texelFetch(normal_tex_current, coord, 0).xyz;
	vec3 normal_current_world = (transform_current * vec4(normal_current_camera, 0.0)).xyz;
	float angle_cos = dot(normal_current_world, normal_prev_world);
	if(angle_cos < angle_cos_threshold)
	{
		StoreNopResidual();
		return;
	}

#ifdef ICP_DEBUG_TEX
	imageStore(debug_out, coord, vec4(dir_world, 43.0));
#endif

	// see K. Low. Linear least-squares optimization for point-to-plane ICP surface registration.

	vec3 n = normal_prev_world;
	vec3 d = vertex_prev_world;
	vec3 s = vertex_current_world;

	StoreResidual(cross(s, n), n, dot(n, dir_world));
}
)glsl";


static const char *reduce_shader_code =
"#version 450 core\n"
"#define COLUMNS " TOSTR(MATRIX_COLUMNS) "\n"
"#define ROWS " TOSTR(MATRIX_ROWS) "\n"
"#line " TOSTR(__LINE__) "\n" R"glsl(

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

uniform uint residuals_count;

layout(std430, binding = 0) buffer residuals_in
{
	float residuals[];
};

layout(std430, binding = 1) buffer matrix_out
{
	float matrix[COLUMNS*ROWS];
};

float GetResidualValue(uvec2 coord)
{
	return residuals[COLUMNS * coord.y + coord.x];
}

void main()
{
	uvec2 coord = gl_GlobalInvocationID.xy;
	float res = 0.0;
	for(uint i=0; i<residuals_count; i++)
	{
		float a = GetResidualValue(uvec2(coord.y, i));
		float b = GetResidualValue(uvec2(coord.x, i));
		if(isnan(a) || isnan(b))
			continue;
		res += a * b;
	}
	matrix[coord.x * ROWS + coord.y] = res;
}
)glsl";

ICP::ICP()
{
	corr_program = CreateComputeShader(corr_shader_code);

	corr_distance_sq_threshold_uniform = glGetUniformLocation(corr_program, "distance_sq_threshold");
	corr_angle_cos_threshold_uniform = glGetUniformLocation(corr_program, "angle_cos_threshold");
	corr_modelview_prev_uniform = glGetUniformLocation(corr_program, "modelview_prev");
	corr_projection_prev_uniform = glGetUniformLocation(corr_program, "projection_prev");
	corr_transform_current_uniform = glGetUniformLocation(corr_program, "transform_current");
	corr_image_width_uniform = glGetUniformLocation(corr_program, "image_width");

	glObjectLabel(GL_PROGRAM, corr_program, -1, "ICP::corr_program");

	glGenBuffers(1, &residuals_buffer);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, residuals_buffer);
	glObjectLabel(GL_BUFFER, residuals_buffer, -1, "ICP::residuals_buffer");
	residuals_buffer_size = 0;

	reduce_program = CreateComputeShader(reduce_shader_code);
	reduce_residuals_count_uniform = glGetUniformLocation(reduce_program, "residuals_count");
	glObjectLabel(GL_PROGRAM, reduce_program, -1, "ICP::reduce_program");

	glGenBuffers(1, &matrix_buffer);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, matrix_buffer);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(float) * MATRIX_COLUMNS * MATRIX_ROWS, nullptr, GL_DYNAMIC_READ);


#ifdef ICP_DEBUG_TEX
	glGenTextures(1, &debug_tex);
	glBindTexture(GL_TEXTURE_2D, debug_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glObjectLabel(GL_TEXTURE, debug_tex, -1, "ICP::debug_tex");
#endif
}

ICP::~ICP()
{
	glDeleteProgram(corr_program);
	glDeleteBuffers(1, &residuals_buffer);
#ifdef ICP_DEBUG_TEX
	glDeleteTextures(1, &debug_tex);
#endif
}

void ICP::SearchCorrespondences(Frame *frame, Renderer *renderer, const CameraTransform &cam_transform_old, CameraTransform *cam_transform_new)
{
	auto residuals_buffer_size_required =
			static_cast<size_t>(frame->GetDepthWidth() * frame->GetDepthHeight())
			* RESIDUAL_COMPONENTS * sizeof(float);
	if (residuals_buffer_size_required != residuals_buffer_size)
	{
		residuals_buffer_size = residuals_buffer_size_required;
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, residuals_buffer);
		glBufferData(GL_SHADER_STORAGE_BUFFER, residuals_buffer_size, nullptr, GL_DYNAMIC_COPY);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

	}

#ifdef ICP_DEBUG_TEX
	if(debug_tex_width != frame->GetDepthWidth() || debug_tex_height != frame->GetDepthHeight())
	{
		debug_tex_width = frame->GetDepthWidth();
		debug_tex_height = frame->GetDepthHeight();
		glBindTexture(GL_TEXTURE_2D, debug_tex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, debug_tex_width, debug_tex_height, 0, GL_RGBA, GL_FLOAT, nullptr);
	}
	glBindImageTexture(0, debug_tex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
#endif

	cam_transform_new->SetTransform(cam_transform_old.GetTransform());

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, residuals_buffer);

	glUseProgram(corr_program);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, renderer->GetVertexTex());
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, renderer->GetNormalTex());
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, frame->GetVertexTex());
	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, frame->GetNormalTex());

	glUniform1f(corr_distance_sq_threshold_uniform, ICP_DISTANCE_THRESHOLD * ICP_DISTANCE_THRESHOLD); // TODO: make user-adjustable
	glUniform1f(corr_angle_cos_threshold_uniform, ICP_ANGLE_COS_TRESHOLD); // TODO: make user-adjustable

	glUniformMatrix4fv(corr_modelview_prev_uniform, 1, GL_FALSE, renderer->GetModelviewMatrix().data());
	glUniformMatrix4fv(corr_projection_prev_uniform, 1, GL_FALSE, renderer->GetProjectionMatrix().data());

	glUniformMatrix4fv(corr_transform_current_uniform, 1, GL_FALSE, cam_transform_new->GetTransform().matrix().data());

	glUniform1i(corr_image_width_uniform, frame->GetDepthWidth());


	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);
	glDispatchCompute(static_cast<GLuint>(frame->GetDepthWidth()), static_cast<GLuint>(frame->GetDepthHeight()), 1);
}

#include <iostream>
#include <Eigen/Dense>

void ICP::SolveMatrix(int residuals_count, CameraTransform *cam_transform_new)
{
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, residuals_buffer);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, matrix_buffer);

	glUseProgram(reduce_program);
	glUniform1ui(reduce_residuals_count_uniform, static_cast<GLuint>(residuals_count));

	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	glDispatchCompute(MATRIX_COLUMNS, MATRIX_ROWS, 1);

	glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
	Eigen::Matrix<float, MATRIX_ROWS, MATRIX_COLUMNS> matrix;
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, matrix_buffer);
	glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(float) * MATRIX_COLUMNS * MATRIX_ROWS, matrix.data());

	std::cout << "Resulting Matrix:\n" << matrix << std::endl;

	Eigen::Matrix<float, MATRIX_ROWS, MATRIX_ROWS> A = matrix.block<MATRIX_ROWS, MATRIX_ROWS>(0, 0);
	Eigen::Matrix<float, MATRIX_ROWS, 1> b = matrix.col(6);

	std::cout << "A:\n" << A << std::endl;
	std::cout << "b:\n" << b << std::endl;

	Eigen::Matrix<float, MATRIX_ROWS, 1> result = A.colPivHouseholderQr().solve(b);

	std::cout << "result:\n" << result << std::endl;
}
