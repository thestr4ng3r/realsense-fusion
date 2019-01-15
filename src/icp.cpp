
#include "icp.h"
#include "frame.h"
#include "camera_transform.h"
#include "shader_common.h"
#include "renderer.h"

#define RESIDUAL_COMPONENTS 7
#define ICP_DISTANCE_THRESHOLD 100.0f
#define ICP_ANGLE_COS_TRESHOLD 0.0f

static const char *corr_shader_code =
R"glsl(
#version 450 core

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

void StoreNopResidual(float v)
{
	StoreResidual(vec3(v), vec3(0.0), 0.0);
}

void main()
{
	// see also https://github.com/chrdiller/KinectFusionLib/blob/master/src/cuda/pose_estimation.cu
	// but we're not building a pyramid of doom

	coord = ivec2(gl_GlobalInvocationID.xy);
	vec3 vertex_current_camera = texelFetch(vertex_tex_current, coord, 0).xyz;
	if(isinf(vertex_current_camera.x))
	{
		StoreNopResidual(1.0);
		return;
	}

	vec3 vertex_current_world = (transform_current * vec4(vertex_current_camera, 1.0)).xyz;
	vec3 vertex_current_camera_prev = (modelview_prev * vec4(vertex_current_world, 1.0)).xyz;
	vec4 vertex_current_image_prev_p = projection_prev * vec4(vertex_current_camera_prev, 1.0);
	vec2 vertex_current_image_prev = vertex_current_image_prev_p.xy / vertex_current_image_prev_p.w;
	if(vertex_current_image_prev.x < 0.0 || vertex_current_image_prev.y < 0.0
		|| vertex_current_image_prev.x > 1.0 || vertex_current_image_prev.y < 1.0
		|| vertex_current_camera_prev.z < 0.0)
	{
		StoreNopResidual(2.0);
		return;
	}

	vec3 vertex_prev_world = texture(vertex_tex_prev, vertex_current_image_prev).xyz;
	if(isinf(vertex_prev_world.x))
	{
		StoreNopResidual(3.0);
		return;
	}

	vec3 dir_world = vertex_current_world - vertex_prev_world;
	float dist_sq = dot(dir_world, dir_world);
	if(dist_sq > distance_sq_threshold)
	{
		StoreNopResidual(4.0);
		return;
	}

	vec3 normal_prev_world = texture(normal_tex_prev, vertex_current_image_prev).xyz;
	vec3 normal_current_camera = texelFetch(normal_tex_current, coord, 0).xyz;
	vec3 normal_current_world = (transform_current * vec4(normal_current_camera, 0.0)).xyz;
	float angle_cos = dot(normal_current_world, normal_prev_world);
	if(angle_cos < angle_cos_threshold)
	{
		StoreNopResidual(5.0);
		return;
	}

	StoreResidual(
		cross(vertex_current_world, normal_prev_world),
		vec3(vertex_current_image_prev, 42.0),
		dot(normal_prev_world, dir_world));
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
	glObjectLabel(GL_BUFFER, residuals_buffer, -1, "ICP::residuals_buffer");
	residuals_buffer_size = 0;
}

ICP::~ICP()
{
	glDeleteProgram(corr_program);
	glDeleteBuffers(1, &residuals_buffer);
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
