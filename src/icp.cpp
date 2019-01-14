
#include "icp.h"
#include "frame.h"
#include "camera_transform.h"
#include "shader_common.h"

static const char *corr_shader_code =
"#version 450 core\n"
#include "glsl_common_projection.inl"
R"glsl(

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

uniform float depth_scale;

layout(binding = 0) uniform sampler2D vertex_tex_current;
layout(binding = 1) uniform sampler2D normal_tex_current;

layout(rgba32f, binding = 0) uniform image2D vertex_out;
layout(rgba32f, binding = 1) uniform image2D normal_out;

void main()
{
}
)glsl";

ICP::ICP()
{
	corr_program = CreateComputeShader(corr_shader_code);
}

ICP::~ICP()
{
	glDeleteProgram(corr_program);
}

void ICP::SearchCorrespondences(Frame *frame, CameraTransform *cam_transform_old, CameraTransform *cam_transform_new)
{
}
