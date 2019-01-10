#include "pc_integrator.h"
#include "GL/glew.h"

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


PC_Integrator::PC_Integrator(GLModel* glModel, Input* input)
{
	this->glModel = glModel;

	this->resolutionX = glModel->GetResolutionX();
	this->resolutionY = glModel->GetResolutionY();
	this->resolutionZ = glModel->GetResolutionZ();

	this->input = input;

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
		R"glsl(

		layout(r32f, binding = 0) uniform image3D  tsdf_tex;
		layout(r32f, binding = 1) uniform image3D  weight_tex;
		layout(binding = 0) uniform sampler2D depth_map;

		uniform vec2 intrinsic_focalLength;
		uniform vec2 intrinsic_center;

		uniform mat4 mvp_matrix;
		uniform vec3 cam_pos;
		uniform mat4 transpose_inv;

		uniform vec3 resolution;
		uniform float cellSize;
		uniform float depth_scale;

		layout (local_size_x = 1, local_size_y = 1, local_size_z=1) in;
		void main() {
			ivec3 xyz = ivec3(gl_GlobalInvocationID.xyz);
			
			vec3 gridPos = TexelToGrid(xyz);

			vec4 v_g = vec4(GridToWorld(gridPos),1.0f);

			vec4 v = transpose_inv * v_g;

			ivec2 p = ivec2(v.x/v.z * intrinsic_focalLength.x + intrinsic_center.x , v.y/v.z * intrinsic_focalLength.y + intrinsic_center.y );

			if( abs(v.x) > 1 || abs(v.y) > 1 )
			{
					//return;
			}
			if( v.z < 0.00001f ) //behind near plane
			{
				//return;
			}

			float sdf = distance( vec4(cam_pos,1) , v_g) - ReadDepth(depth_map, p, depth_scale) ;

			//float tsdf = clamp(sdf, -max_dist, max_dist);

			float tsdf;

			if (sdf > 0)
			{
				float tsdf = min(1.0f, sdf / 0.000001);
			}
			else 
			{
				float tsdf = max(-1.0f, sdf/ -0.000001);
			}		

			float w_last = imageLoad(weight_tex, xyz).x ; 
			float tsdf_last = imageLoad(tsdf_tex, xyz).x;

			float max_weight = 1.0 / 0.0;   //  = inf 
			float w_now = min (max_weight , w_last +1); 
			
			float tsdf_avg = ( tsdf_last * w_last + tsdf * w_now ) / w_now ;

			imageStore(tsdf_tex, xyz, vec4(tsdf_avg,0.0,0.0,0.0));
			imageStore(weight_tex, xyz, vec4(w_now,0.0,0.0,0.0));
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

	intrinsic_center_uniform = glGetUniformLocation(progHandle, "intrinsic_center");
	intrinsic_focalLength_uniform = glGetUniformLocation(progHandle, "intrinsic_focalLength");
	mvp_matrix_uniform = glGetUniformLocation(progHandle, "mvp_matrix");
	cam_pos_uniform = glGetUniformLocation(progHandle, "cam_pos");
	transposeInv_uniform = glGetUniformLocation(progHandle, "transpose_inv");
	tsdf_tex_uniform = glGetUniformLocation(progHandle, "tsdf_tex");
	weight_tex_uniform = glGetUniformLocation(progHandle, "weight_tex");
	depth_map_uniform = glGetUniformLocation(progHandle, "depth_map");
	resolution_uniform = glGetUniformLocation(progHandle, "resolution");
	cellSize_uniform = glGetUniformLocation(progHandle, "cellSize");
	depth_scale_uniform = glGetUniformLocation(progHandle, "depth_scale");

	glUniform1i(tsdf_tex_uniform, 0); //Image Unit 0
	glUniform1i(weight_tex_uniform, 1);
	glUniform1i(depth_map_uniform, 0);//Texture Unit 0

	return progHandle;
}

void PC_Integrator::integrate(Frame* frame)
{

	int depthResX = frame->GetDepthWidth();
	int depthResY = frame->GetDepthHeight();
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, frame->GetDepthTex());

	// TODO DELOCALIZE CAM POS & PROJECT MATRIX OUT OF PC_INTEGRATOR || Check if MVP matrix is correct
	Eigen::Vector3f cam_pos(0.0f, 0.0f, 3.0f);

	Eigen::Affine3f modelview = Eigen::Affine3f::Identity();
	modelview.translate(-cam_pos);
	//modelview.rotate(Eigen::AngleAxisf(0.25f * M_PI, Eigen::Vector3f::UnitX()));

	Eigen::Matrix4f mvp_matrix;
	mvp_matrix = PerspectiveMatrix<float>(80.0f, (float)depthResX / (float)depthResY, 0.1f, 100.0f) * modelview.matrix();

	Eigen::Matrix4f transpose;   //ToDo Integrate Frame transpose here
	transpose = Eigen::Matrix4f::Identity();
	//transpose = modelview.matrix().inverse();

	glUseProgram(this->computeHandle);

	// UNIFORMS
	// ToDo : Get Intrinsics
	glUniform2f(intrinsic_center_uniform, input->GetPpx(), input->GetPpy());
	glUniform2f(intrinsic_focalLength_uniform, input->GetFx(), input->GetFy());
	glUniform1f(cellSize_uniform, cellSize);
	glUniform3f(resolution_uniform, resolutionX, resolutionY, resolutionZ);
	glUniformMatrix4fv(mvp_matrix_uniform, 1, GL_FALSE, mvp_matrix.data());
	glUniform3fv(cam_pos_uniform, 1, cam_pos.data());
	glUniformMatrix4fv(transposeInv_uniform, 1, GL_FALSE, transpose.data());
	glUniform1f(depth_scale_uniform, frame->GetDepthScale());
	glBindImageTexture(0, glModel->GetTSDFTex(), 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32F);
	glBindImageTexture(1, glModel->GetWeightTex(), 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32F);

	//todo loop over slide
	glBindBufferBase(GL_UNIFORM_BUFFER, 0, this->glModel->GetParamsBuffer());
	glDispatchCompute(resolutionX, resolutionY, resolutionZ);

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
