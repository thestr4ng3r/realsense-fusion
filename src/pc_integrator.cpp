#include "pc_integrator.h"
#include "camera_transform.h"
#include "GL/glew.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#define MAX_WEIGHT 256

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


PC_Integrator::PC_Integrator(GLModel* glModel)
{
	this->glModel = glModel;

	this->resolutionX = glModel->GetResolutionX();
	this->resolutionY = glModel->GetResolutionY();
	this->resolutionZ = glModel->GetResolutionZ();

	this->computeHandle = genComputeProg();
}

PC_Integrator::~PC_Integrator()
{
}

GLuint PC_Integrator::genComputeProg()
{
	GLuint progHandle = glCreateProgram();
	GLuint cs = glCreateShader(GL_COMPUTE_SHADER);

	static const char *csSrc =
		"#version 450 core\n"
		#include "glsl_common_grid.inl"
		#include "glsl_common_depth.inl"
		#include "glsl_common_projection.inl"
		R"glsl(

		layout(r32f, binding = 0) uniform image3D  tsdf_tex;
		layout(r16ui, binding = 1) uniform uimage3D  weight_tex;
		layout(binding = 0) uniform usampler2D depth_map;

		uniform mat4 cam_modelview;
		uniform vec3 cam_pos;

		uniform float cellSize;
		uniform float depth_scale;

		uniform float max_truncation;
		uniform float min_truncation;
		uniform uint max_weight;

		layout (local_size_x = 1, local_size_y = 1, local_size_z=1) in;
		void main() {
			for(uint z=0; z<grid_params.res.z; z++)
			{
				ivec3 xyz = ivec3(gl_GlobalInvocationID.xy, z);

				vec3 gridPos = TexelToGrid(xyz);

				vec4 v_g = vec4(GridToWorld(gridPos),1.0f);

				vec4 v = cam_modelview * v_g;

				ivec2 p = ivec2(ProjectCameraToImage(v.xyz));

				ivec2 depth_res = textureSize(depth_map, 0);
				if( p.x < 0 || p.x > depth_res.x || p.y < 0 || p.y > depth_res.y  )
				{
					continue;
				}

				if( abs(v.x) > 1 || abs(v.y) > 1 )
				{
					//continue;
				}

				float depth = ReadDepth(depth_map, p, depth_scale);
				if(depth == 0)
				{
					continue;
				}

				float sdf = depth - distance(vec4(cam_pos,1) , v_g);

				float tsdf = clamp(sdf, min_truncation, max_truncation);

				if (sdf > 0)
				{
					float ttsdf = min(sdf, max_truncation);
				}
				else
				{
					float ttsdf = max(sdf, min_truncation);
				}

				uint w_last = imageLoad(weight_tex, xyz).x;
				float tsdf_last = imageLoad(tsdf_tex, xyz).x;

				uint w_now = min(max_weight, w_last + 1);

				float tsdf_avg = (tsdf_last * w_last + tsdf * w_now) / (w_now + w_last);

				imageStore(tsdf_tex, xyz, vec4(tsdf,0.0,0.0,0.0));
				imageStore(weight_tex, xyz, uvec4(w_now,0.0,0.0,0.0));
			}
		}		
	    )glsl";

	glShaderSource(cs, 1, &csSrc, NULL);
	glCompileShader(cs);
	int rvalue;
	glGetShaderiv(cs, GL_COMPILE_STATUS, &rvalue);
	if (!rvalue)
	{
		fprintf(stderr, "Error in compiling the compute shader\n");
		GLchar log[10240];
		GLsizei length;
		glGetShaderInfoLog(cs, 10239, &length, log);
		fprintf(stderr, "Compiler log:\n%s\n", log);
		exit(40);
	}
	glAttachShader(progHandle, cs);

	glLinkProgram(progHandle);
	glGetProgramiv(progHandle, GL_LINK_STATUS, &rvalue);
	if (!rvalue) {
		fprintf(stderr, "Error in linking compute shader program\n");
		GLchar log[10240];
		GLsizei length;
		glGetProgramInfoLog(progHandle, 10239, &length, log);
		fprintf(stderr, "Linker log:\n%s\n", log);
		exit(41);
	}
	glUseProgram(progHandle);

	cam_modelview_uniform = glGetUniformLocation(progHandle, "cam_modelview");
	cam_pos_uniform = glGetUniformLocation(progHandle, "cam_pos");
	tsdf_tex_uniform = glGetUniformLocation(progHandle, "tsdf_tex");
	weight_tex_uniform = glGetUniformLocation(progHandle, "weight_tex");
	depth_scale_uniform = glGetUniformLocation(progHandle, "depth_scale");
	max_truncation_uniform = glGetUniformLocation(progHandle, "max_truncation");
	min_truncation_uniform = glGetUniformLocation(progHandle, "min_truncation");
	max_weight_uniform = glGetUniformLocation(progHandle, "max_weight");

	glUniform1i(tsdf_tex_uniform, 0); //Image Unit 0
	glUniform1i(weight_tex_uniform, 1);

	return progHandle;
}

void PC_Integrator::integrate(Frame *frame, CameraTransform *camera_transform)
{
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, frame->GetDepthTex());


	Eigen::Vector3f cam_pos = camera_transform->GetTransform().translation();
	Eigen::Matrix4f modelview = camera_transform->GetModelView();

	glUseProgram(this->computeHandle);

	// UNIFORMS
	glUniform3fv(cam_pos_uniform, 1, cam_pos.data());
	glUniformMatrix4fv(cam_modelview_uniform, 1, GL_FALSE, modelview.data());
	glUniform1f(depth_scale_uniform, frame->GetDepthScale());
	glUniform1f(max_truncation_uniform, glModel->GetMaxTruncation());
	glUniform1f(min_truncation_uniform, glModel->GetMinTruncation());
	glUniform1ui(max_weight_uniform, MAX_WEIGHT);
	glBindImageTexture(0, glModel->GetTSDFTex(), 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32F);
	glBindImageTexture(1, glModel->GetWeightTex(), 0, GL_TRUE, 0, GL_READ_WRITE, GL_R16UI);
	glBindBufferBase(GL_UNIFORM_BUFFER, 0, this->glModel->GetParamsBuffer());
	glBindBufferBase(GL_UNIFORM_BUFFER, 1, frame->GetCameraIntrinsicsBuffer());

	glDispatchCompute(resolutionX, resolutionY, 1);

}

GLuint PC_Integrator::genTexture2D(int resolutionX, int resolutionY, float* data)
{
	GLuint texHandle;
	glGenTextures(1, &texHandle);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, texHandle);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, resolutionX, resolutionY, 0, GL_RED, GL_FLOAT, data); //add here input
	return texHandle;
}
