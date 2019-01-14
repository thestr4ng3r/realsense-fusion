
#include "icp.h"
#include "frame.h"
#include "camera_transform.h"
#include "shader_common.h"
#include "renderer.h"

#define RESIDUAL_COMPONENTS 6

static const char *corr_shader_code =
"#version 450 core\n"
#include "glsl_common_projection.inl"
R"glsl(

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

uniform mat4 modelview_prev;

layout(binding = 0) uniform sampler2D vertex_tex_prev;
layout(binding = 1) uniform sampler2D normal_tex_prev;

uniform mat4 transform_current;

layout(binding = 2) uniform sampler2D vertex_tex_current;
layout(binding = 3) uniform sampler2D normal_tex_current;


layout(std430, binding = 0) buffer residuals_out
{
	float residuals[];
};

ivec2 coord;

void StoreResidual(vec3 vertex, vec3 normal)
{
	uint base = 6 * uint(coord.y * camera_intrinsics.res.x + coord.x);
	residuals[base+0] = vertex.x;
	residuals[base+1] = vertex.y;
	residuals[base+2] = vertex.z;
	residuals[base+3] = normal.x;
	residuals[base+4] = normal.y;
	residuals[base+5] = normal.z;
}

void StoreNopResidual()
{
	StoreResidual(vec3(0.0), vec3(0.0));
}

void main()
{
	coord = ivec2(gl_GlobalInvocationID.xy);
	vec3 vertex_current_camera = texelFetch(vertex_tex_current, coord, 0).xyz;
	if(isinf(vertex_current.x))
	{
		StoreNopResidual();
		return;
	}
	vec3 normal_current = texelFetch(normal_tex_current, coord, 0).xyz;

	vec3 vertex_current_world = transform_current * vertex_current_camera;
	vec3 vertex_current_camera_prev = modelview_prev * vertex_current_world;
	vec2 vertex_current_image_prev = ProjectCameraToImage(vertex_current_camera_prev) / vec2(camera_instrinsics.res);
	if(vertex_current_image_prev.x < 0.0 || vertex_current_image_prev.y < 0.0
		|| vertex_current_image_prev.x > 1.0 || vertex_current_image_prev.y < 1.0
		|| vertex_current_camera_prev.z < 0.0)
	{
		StoreNopResidual();
		return;
	}

	vec3 vertex_prev_camera = texture(vertex_tex_prev, vertex_current_image_prev).xyz;

	// https://github.com/chrdiller/KinectFusionLib/blob/master/src/cuda/pose_estimation.cu#L94

	StoreResidual(vertex_current, normal_current);
}
)glsl";

ICP::ICP()
{
	corr_program = CreateComputeShader(corr_shader_code);
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

	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);
	glDispatchCompute(static_cast<GLuint>(frame->GetDepthWidth()), static_cast<GLuint>(frame->GetDepthHeight()), 1);
}
