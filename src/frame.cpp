
#include "frame.h"
#include "shader_common.h"

static const char *process_shader_code =
"#version 450 core\n"
#include "glsl_common_depth.inl"
#include "glsl_common_projection.inl"
R"glsl(

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

uniform float depth_scale;

layout(binding = 0) uniform usampler2D depth_tex;

layout(rgba32f, binding = 0) uniform image2D vertex_out;
layout(rgba32f, binding = 1) uniform image2D normal_out;

vec3 VertexForCoords(ivec2 coords, inout float depth)
{
	depth = ReadDepth(depth_tex, coords, depth_scale);
	return DeprojectImageToCamera(vec2(coords), depth);
}

void main()
{
	ivec2 coords = ivec2(gl_GlobalInvocationID.xy);

	float depth = 0.0;
	vec3 pos = VertexForCoords(coords, depth);

	vec3 normal;
	if(depth != 0.0)
	{
		vec3 dx = VertexForCoords(coords + ivec2(1, 0), depth);
		if(depth == 0.0)
			dx = vec3(1.0, 0.0, 0.0);
		else
			dx -= pos;

		vec3 dy = VertexForCoords(coords + ivec2(0, -1), depth);
		if(depth == 0.0)
			dy = vec3(0.0, 1.0, 0.0);
		else
			dy -= pos;

		normal = normalize(cross(dx, dy));
	}
	else
	{
		pos = vec3(1.0 / 0.0);
		normal = vec3(1.0 / 0.0);
	}

	imageStore(vertex_out, coords, vec4(pos, 0.0));
	imageStore(normal_out, coords, vec4(normal, 0.0));
}
)glsl";

Frame::Frame() :
	//cloud(new pcl::PointCloud<pcl::PointXYZ>)
	depth_width(0),
	depth_height(0),
	depth_scale(1.0f)
{
	glGenTextures(1, &depth_tex);
	glBindTexture(GL_TEXTURE_2D, depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glGenTextures(1, &vertex_tex);
	glBindTexture(GL_TEXTURE_2D, vertex_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glGenTextures(1, &normal_tex);
	glBindTexture(GL_TEXTURE_2D, normal_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glGenTextures(1, &color_tex);
	glBindTexture(GL_TEXTURE_2D, color_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);


	glGenBuffers(1, &camera_intrinsics_buffer);
	glGenBuffers(1, &camera_intrinsics_colorbuffer);

	process_program = CreateComputeShader(process_shader_code);
	depth_scale_uniform = glGetUniformLocation(process_program, "depth_scale");
}

Frame::~Frame()
{
	glDeleteTextures(1, &depth_tex);
	glDeleteTextures(1, &vertex_tex);
	glDeleteTextures(1, &normal_tex);
	glDeleteTextures(1, &color_tex);
	glDeleteBuffers(1, &camera_intrinsics_buffer);
	glDeleteBuffers(1, &camera_intrinsics_colorbuffer);
	glDeleteProgram(process_program);
}

void Frame::SetDepthMap(int width, int height, GLushort *data, float depth_scale, const Eigen::Vector2f &focal_length, const Eigen::Vector2f &center)
{
	this->depth_scale = depth_scale;
	glBindTexture(GL_TEXTURE_2D, depth_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R16UI, width, height, 0, GL_RED_INTEGER, GL_UNSIGNED_SHORT, data);

	if(width != depth_width || height != depth_height)
	{
		glBindTexture(GL_TEXTURE_2D, vertex_tex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
		glObjectLabel(GL_TEXTURE, vertex_tex, -1, "Frame::vertex_tex");

		glBindTexture(GL_TEXTURE_2D, normal_tex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
		glObjectLabel(GL_TEXTURE, normal_tex, -1, "Frame::normal_tex");

		this->depth_width = width;
		this->depth_height = height;
	}

	intrinsics_focal_length = focal_length;
	intrinsics_center = center;

	uint32_t buf[8];
	*((float *)(buf + 0)) = intrinsics_focal_length.x();
	*((float *)(buf + 1)) = intrinsics_focal_length.y();
	*((float *)(buf + 2)) = intrinsics_center.x();
	*((float *)(buf + 3)) = intrinsics_center.y();
	*(buf + 4) = (uint32_t)depth_width;
	*(buf + 5) = (uint32_t)depth_height;
	*(buf + 6) = 0;
	*(buf + 7) = 0;

	glBindBuffer(GL_UNIFORM_BUFFER, camera_intrinsics_buffer);
	glBufferData(GL_UNIFORM_BUFFER, sizeof(buf), buf, GL_DYNAMIC_DRAW);
}

void Frame::SetColorMap(int width, int height, GLushort *data, const Eigen::Vector2f &focal_length, const Eigen::Vector2f &center)
{
	glBindTexture(GL_TEXTURE_2D, color_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

	intrinsics_color_focal_length = focal_length;
	intrinsics_color_center = center;

	uint32_t buf[8];
	*((float *)(buf + 0)) = intrinsics_color_focal_length.x();
	*((float *)(buf + 1)) = intrinsics_color_focal_length.y();
	*((float *)(buf + 2)) = intrinsics_color_center.x();
	*((float *)(buf + 3)) = intrinsics_color_center.y();
	*(buf + 4) = (uint32_t)width;
	*(buf + 5) = (uint32_t)height;
	*(buf + 6) = 0;
	*(buf + 7) = 0;

	glBindBuffer(GL_UNIFORM_BUFFER, camera_intrinsics_colorbuffer);
	glBufferData(GL_UNIFORM_BUFFER, sizeof(buf), buf, GL_DYNAMIC_DRAW);
}

void Frame::ProcessFrame()
{
	if(depth_width == 0 || depth_height == 0)
		return;

	glUseProgram(process_program);

	glUniform1f(depth_scale_uniform, depth_scale);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, depth_tex);

	glBindImageTexture(0, vertex_tex, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);
	glBindImageTexture(1, normal_tex, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA32F);

	glBindBufferBase(GL_UNIFORM_BUFFER, 1, camera_intrinsics_buffer);

	glDispatchCompute(static_cast<GLuint>(depth_width), static_cast<GLuint>(depth_height), 1);
}