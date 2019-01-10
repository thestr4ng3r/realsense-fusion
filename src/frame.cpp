
#include "frame.h"
#include "shader_common.h"

static const char *process_shader_code =
"#version 450 core\n"
#include "glsl_common_depth.inl"
R"glsl(

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

uniform float depth_scale;

layout(binding = 0) uniform usampler2D depth_tex;

layout(rgba32f, binding = 0) uniform image2D vertex_out;
layout(rgba32f, binding = 1) uniform image2D normal_out;

void main()
{
	ivec2 coords = ivec2(gl_GlobalInvocationID.xy);
	float depth = ReadDepth(depth_tex, coords, depth_scale);
	imageStore(vertex_out, coords, vec4(depth, 0.0, 0.0, 0.0));
	imageStore(normal_out, coords, vec4(0.0, 0.0, 1.0, 0.0));
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
	glBindTexture(GL_TEXTURE_2D, depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glGenTextures(1, &normal_tex);
	glBindTexture(GL_TEXTURE_2D, depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	process_program = CreateComputeShader(process_shader_code);
	depth_scale_uniform = glGetUniformLocation(process_program, "depth_scale");
}

Frame::~Frame()
{
	glDeleteTextures(1, &depth_tex);
	glDeleteTextures(1, &vertex_tex);
	glDeleteTextures(1, &normal_tex);
}

void Frame::SetDepthMap(int width, int height, GLushort *data, float depth_scale)
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
}

void Frame::ProcessFrame()
{
	if(depth_width == 0 || depth_height == 0)
		return;

	glUseProgram(process_program);

	glUniform1f(depth_scale_uniform, depth_scale);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, depth_tex);

	glBindImageTexture(0, vertex_tex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
	glBindImageTexture(1, normal_tex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);

	glDispatchCompute(static_cast<GLuint>(depth_width), static_cast<GLuint>(depth_height), 1);

	glFinish();
	glFlush();
	glFinish();
	glFlush();
}