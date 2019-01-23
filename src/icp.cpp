
#include <icp.h>

#include "icp.h"
#include "frame.h"
#include "camera_transform.h"
#include "shader_common.h"
#include "renderer.h"

#define RESIDUAL_COMPONENTS 7
#define MATRIX_COLUMNS RESIDUAL_COMPONENTS
#define MATRIX_ROWS (RESIDUAL_COMPONENTS-1)

#define STRHELPER(x) #x
#define TOSTR(x) STRHELPER(x)

#define CORR_LOCAL_SIZE 32

static const char *corr_shader_code =
"#version 450 core\n"
#ifdef ICP_DEBUG_TEX
"#define ICP_DEBUG_TEX\n"
#endif
"#define RESIDUAL_COMPONENTS " TOSTR(RESIDUAL_COMPONENTS) "\n"
"#define COLUMNS " TOSTR(MATRIX_COLUMNS) "\n"
"#define ROWS " TOSTR(MATRIX_ROWS) "\n"
"#define LOCAL_SIZE " TOSTR(CORR_LOCAL_SIZE) "\n"
"#define LOCAL_SIZE_TOTAL (LOCAL_SIZE*LOCAL_SIZE)\n"
"#line " TOSTR(__LINE__) "\n" R"glsl(

layout(local_size_x = LOCAL_SIZE, local_size_y = LOCAL_SIZE, local_size_z = 1) in;

uniform mat4 modelview_prev;
uniform mat4 projection_prev;

layout(binding = 0) uniform sampler2D vertex_tex_prev;
layout(binding = 1) uniform sampler2D normal_tex_prev;

uniform mat4 transform_current;

layout(binding = 2) uniform sampler2D vertex_tex_current;
layout(binding = 3) uniform sampler2D normal_tex_current;

uniform float distance_sq_threshold;
uniform float angle_cos_threshold;

uniform uvec2 image_res;

layout(std430, binding = 0) buffer residuals_out
{
	float residuals[];
};

#ifdef ICP_DEBUG_TEX
layout(rgba32f, binding = 0) uniform image2D debug_out;
#endif

float[RESIDUAL_COMPONENTS] Residual(vec3 a, vec3 b, float c)
{
	float r[RESIDUAL_COMPONENTS];
	r[0] = a.x;
	r[1] = a.y;
	r[2] = a.z;
	r[3] = b.x;
	r[4] = b.y;
	r[5] = b.z;
	r[6] = c;
	return r;
}

float[RESIDUAL_COMPONENTS] NopResidual()
{
	return Residual(vec3(0.0), vec3(0.0), 0.0);
}

float[RESIDUAL_COMPONENTS] CreateResidual(ivec2 coord)
{
	// see also https://github.com/chrdiller/KinectFusionLib/blob/master/src/cuda/pose_estimation.cu
	// but we're not building a pyramid of doom

#ifdef ICP_DEBUG_TEX
	imageStore(debug_out, coord, vec4(-1.0));
#endif

	if(coord.x >= image_res.x || coord.y >= image_res.y)
		return NopResidual();

	vec3 vertex_current_camera = texelFetch(vertex_tex_current, coord, 0).xyz;
	if(isinf(vertex_current_camera.x))
		return NopResidual();

	vec3 vertex_current_world = (transform_current * vec4(vertex_current_camera, 1.0)).xyz;
	vec3 vertex_current_camera_prev = (modelview_prev * vec4(vertex_current_world, 1.0)).xyz;
	vec4 vertex_current_image_prev_p = projection_prev * vec4(vertex_current_camera_prev, 1.0);
	vec2 vertex_current_image_prev = (vertex_current_image_prev_p.xy / vertex_current_image_prev_p.w) * 0.5 + 0.5;
	if(vertex_current_image_prev.x < 0.0 || vertex_current_image_prev.y < 0.0
		|| vertex_current_image_prev.x > 1.0 || vertex_current_image_prev.y > 1.0
		|| vertex_current_camera_prev.z > 0.0)
	{
		return NopResidual();
	}

	vec3 vertex_prev_world = texture(vertex_tex_prev, vertex_current_image_prev).xyz;
	if(isinf(vertex_prev_world.x))
		return NopResidual();

	vec3 dir_world = vertex_prev_world - vertex_current_world;
	float dist_sq = dot(dir_world, dir_world);
	if(dist_sq > distance_sq_threshold)
		return NopResidual();

	vec3 normal_prev_world = texture(normal_tex_prev, vertex_current_image_prev).xyz;
	vec3 normal_current_camera = texelFetch(normal_tex_current, coord, 0).xyz;
	vec3 normal_current_world = (transform_current * vec4(normal_current_camera, 0.0)).xyz;
	float angle_cos = dot(normal_current_world, normal_prev_world);
	if(angle_cos < angle_cos_threshold)
		return NopResidual();

	// see K. Low. Linear least-squares optimization for point-to-plane ICP surface registration.

	vec3 n = normal_prev_world;
	vec3 d = vertex_prev_world;
	vec3 s = vertex_current_world;

#ifdef ICP_DEBUG_TEX
	imageStore(debug_out, coord, vec4(dir_world, 1.0));
#endif

	return Residual(cross(s, n), n, dot(n, dir_world));
}

shared float reduce_buf_shared[LOCAL_SIZE_TOTAL];

void main()
{
	ivec2 coord_global = ivec2(gl_GlobalInvocationID.xy);
	float[RESIDUAL_COMPONENTS] residual_own = CreateResidual(coord_global);

	for(uint i=0; i<RESIDUAL_COMPONENTS; i++)
	{
		if(isnan(residual_own[i]) || isinf(residual_own[i]))
		{
			residual_own = NopResidual();
			break;
		}
	}

	uint local_id_flat = gl_LocalInvocationID.y * LOCAL_SIZE + gl_LocalInvocationID.x;
	for(uint row=0; row<ROWS; row++)
	{
		for(uint col=0; col<COLUMNS; col++)
		{
			float v = residual_own[col] * residual_own[row];
			reduce_buf_shared[local_id_flat] = v;

			memoryBarrierShared();
			barrier();

			// Tree-based summing

#if LOCAL_SIZE_TOTAL >= 1024
			if(local_id_flat < 512)
				reduce_buf_shared[local_id_flat] = v = v + reduce_buf_shared[local_id_flat + 512];
			memoryBarrierShared();
			barrier();
#endif

#if LOCAL_SIZE_TOTAL >= 512
			if(local_id_flat < 256)
				reduce_buf_shared[local_id_flat] = v = v + reduce_buf_shared[local_id_flat + 256];
			memoryBarrierShared();
			barrier();
#endif

#if LOCAL_SIZE_TOTAL >= 256
			if(local_id_flat < 128)
				reduce_buf_shared[local_id_flat] = v = v + reduce_buf_shared[local_id_flat + 128];
			memoryBarrierShared();
			barrier();
#endif

#if LOCAL_SIZE_TOTAL >= 128
			if(local_id_flat < 64)
				reduce_buf_shared[local_id_flat] = v = v + reduce_buf_shared[local_id_flat + 64];
			memoryBarrierShared();
			barrier();
#endif

#if LOCAL_SIZE_TOTAL >= 64
			if(local_id_flat < 32)
				reduce_buf_shared[local_id_flat] = v = v + reduce_buf_shared[local_id_flat + 32];
			memoryBarrierShared();
			barrier();
#endif

#if LOCAL_SIZE_TOTAL >= 32
			if(local_id_flat < 16)
				reduce_buf_shared[local_id_flat] = v = v + reduce_buf_shared[local_id_flat + 16];
			memoryBarrierShared();
			barrier();
#endif

#if LOCAL_SIZE_TOTAL >= 16
			if(local_id_flat < 8)
				reduce_buf_shared[local_id_flat] = v = v + reduce_buf_shared[local_id_flat + 8];
			memoryBarrierShared();
			barrier();
#endif

#if LOCAL_SIZE_TOTAL >= 8
			if(local_id_flat < 4)
				reduce_buf_shared[local_id_flat] = v = v + reduce_buf_shared[local_id_flat + 4];
			memoryBarrierShared();
			barrier();
#endif

#if LOCAL_SIZE_TOTAL >= 4
			if(local_id_flat < 2)
				reduce_buf_shared[local_id_flat] = v = v + reduce_buf_shared[local_id_flat + 2];
			memoryBarrierShared();
			barrier();
#endif

#if LOCAL_SIZE_TOTAL >= 2
			if(local_id_flat < 1)
				v += reduce_buf_shared[local_id_flat + 1];
#endif

			if(local_id_flat == 0)
			{
				uint base = ROWS * COLUMNS * uint(gl_WorkGroupID.y * gl_NumWorkGroups.x + gl_WorkGroupID.x);
				residuals[base + row * COLUMNS + col] = v;
			}
		}
	}
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
	corr_image_res_uniform = glGetUniformLocation(corr_program, "image_res");

	glObjectLabel(GL_PROGRAM, corr_program, -1, "ICP::corr_program");

	glGenBuffers(1, &residuals_buffer);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, residuals_buffer);
	glObjectLabel(GL_BUFFER, residuals_buffer, -1, "ICP::residuals_buffer");
	residuals_count = 0;

	reduce_program = CreateComputeShader(reduce_shader_code);
	reduce_residuals_count_uniform = glGetUniformLocation(reduce_program, "residuals_count");
	glObjectLabel(GL_PROGRAM, reduce_program, -1, "ICP::reduce_program");

	glGenBuffers(1, &matrix_buffer);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, matrix_buffer);
	glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(float) * MATRIX_COLUMNS * MATRIX_ROWS, nullptr, GL_DYNAMIC_READ);

	distance_threshold = 0.1f;
	angle_threshold = 0.5f;

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
	unsigned int width_global = (static_cast<unsigned int>(frame->GetDepthWidth()) + CORR_LOCAL_SIZE - 1) / CORR_LOCAL_SIZE;
	unsigned int height_global = (static_cast<unsigned int>(frame->GetDepthHeight()) + CORR_LOCAL_SIZE - 1) / CORR_LOCAL_SIZE;

	auto residuals_count_required = width_global * height_global * (RESIDUAL_COMPONENTS - 1);

	if (residuals_count != residuals_count_required)
	{
		residuals_count = residuals_count_required;
		size_t residuals_buffer_size = residuals_count_required * RESIDUAL_COMPONENTS * sizeof(float);
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

	glUniform1f(corr_distance_sq_threshold_uniform, distance_threshold * distance_threshold);
	glUniform1f(corr_angle_cos_threshold_uniform, angle_threshold);

	glUniformMatrix4fv(corr_modelview_prev_uniform, 1, GL_FALSE, renderer->GetModelviewMatrix().data());
	glUniformMatrix4fv(corr_projection_prev_uniform, 1, GL_FALSE, renderer->GetProjectionMatrix().data());

	glUniformMatrix4fv(corr_transform_current_uniform, 1, GL_FALSE, cam_transform_new->GetTransform().matrix().data());

	glUniform2ui(corr_image_res_uniform, static_cast<GLuint>(frame->GetDepthWidth()), static_cast<GLuint>(frame->GetDepthHeight()));


	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);
	glDispatchCompute(width_global, height_global, 1);
}

#include <iostream>
#include <Eigen/Dense>

void ICP::SolveMatrix(CameraTransform *cam_transform_new)
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

	Eigen::Matrix<float, MATRIX_ROWS, MATRIX_ROWS> A = matrix.block<MATRIX_ROWS, MATRIX_ROWS>(0, 0);
	Eigen::Matrix<float, MATRIX_ROWS, 1> b = matrix.col(6);

	// TODO: learn what this actually means, currently copied from https://github.com/chrdiller/KinectFusionLib/blob/master/src/pose_estimation.cpp#L54
	if(A.determinant() < 100000 /*1e-15*/ || std::isnan(A.determinant()))
		return;

	Eigen::Matrix<float, MATRIX_ROWS, 1> result = A.colPivHouseholderQr().solve(b);

	Eigen::Affine3f transform = cam_transform_new->GetTransform();
	transform.rotate(Eigen::AngleAxisf(result(0), Eigen::Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf(result(1), Eigen::Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(result(2), Eigen::Vector3f::UnitZ()));
	transform.translate(result.tail<3>());
	cam_transform_new->SetTransform(transform);
}
