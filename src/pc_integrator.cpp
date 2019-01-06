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
		R"glsl(
		#version 450 core

		uniform image3D  tsdf_tex;
		uniform image3D  weight_tex;
		uniform sampler2D depth_map

		uniform vec2 intrinsic_focalLength;
		uniform vec2 intrinsic_center;

		uniform mat4 mvp_matrix;
		uniform vec3 cam_pos;
		uniform mat4 transpose_inv;

		uniform vec3 resolution;
		uniform float cellSize;

		layout (local_size_x = 1, local_size_y = 1, local_size_z=1) in;
		void main() {
			ivec3 xyz = ivec(gl_globalInvocationID.xyz);
			
			float x_cell = float(x) - resolution.x / 2.0f) * cellSize;
			float y_cell = float(y) - resolution.y / 2.0f) * cellSize;
			float z_cell = float(z) - resolution.z / 2.0f) * cellSize;

			//shift into cell center
			x_cell += cellSize / 2.0f;
			y_cell += cellSize / 2.0f;
			z_cell += cellSize / 2.0f;

			vec4 v_g = vec3(x_cell, y_cell, z_cell, 1.0f);

			vec4 v = transpose_inv * v_g;

			vec2 p = vec2(v.x/v.z * intrinsic_focalLength.x + intrinsic_center.x , v.y/v.z * intrinsic_focalLength.y + intrinsic_center.y );

			discard p if v not in frustrum 
			vec4 v_hc = mvp_matrix * v;
			if( abs(v_hc.x) > 1 || abs(v_hc.y) > 1 || abs(v_hc.z) > 1 )
			{
					return;
			}

			float sdf = distance( cam_pos - v_g ) - texture2D(depth_map, p) ;

			float tsdf;

			if (sdf > 0)
			{
				float tsdf = min(1, sdf / 0.000001);
			}
			else 
			{
				float tsdf = max(-1, sdf/ -0.000001);
			}		

			float w_last = imageLoad(weight_tex, xyz) ; 
			float tsdf_last = imageLoad(tsdf_tex, xyz);

			max_weight = 1.0 / 0.0;   //  = inf 
			w_now = min (max_weight , w_last +1); 
			
			float tsdf_avg = ( tsdf_last * w_last + tsdf * w_now ) / w_now;

			imageStore(tsdf_tex, xyz, tsdf_avg);
			imageStore(weight_tex, xyz, w_now);
		}		
	    )glsl";


	glShaderSource(cs, 2, &csSrc, NULL);
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

	glUniform1i(tsdf_tex_uniform, 0); //Image Unit 0
	glUniform1i(weight_tex_uniform, 1);
	glUniform1i(depth_map_uniform, 0);//Texture Unit 0

	return progHandle;
}

void PC_Integrator::integrate(Frame &frame)
{
	// TODO : // Access Depth Image from Camera Input class
	int depthResX = frame.GetDepthWidth();
	int depthResY = frame.GetDepthHeight();
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, frame.GetDepthTex());

	// TODO DELOCALIZE CAM POS & PROJECT MATRIX OUT OF PC_INTEGRATOR || Check if MVP matrix is correct
	Eigen::Vector3f cam_pos(0.0f, 0.0f, 3.0f);

	Eigen::Affine3f modelview = Eigen::Affine3f::Identity();
	modelview.translate(-cam_pos);
	//modelview.rotate(Eigen::AngleAxisf(0.25f * M_PI, Eigen::Vector3f::UnitX()));

	Eigen::Matrix4f mvp_matrix;
	mvp_matrix = PerspectiveMatrix<float>(80.0f, (float)depthResX / (float)depthResY, 0.1f, 100.0f) * modelview.matrix();

	Eigen::Matrix4f transpose;   //ToDo Integrate Frame transpose here
	transpose = Eigen::Matrix4f::Identity();

	glUseProgram(this->computeHandle);

	// UNIFORMS
	// ToDo : Get Intrinsics
	//glUniform2f(intrinsic_center, )
	//glUniform2f(intrinsic_focalLength, )
	glUniform1f(cellSize_uniform, cellSize);
	glUniform3f(resolution_uniform, resolutionX, resolutionY, resolutionZ);
	glUniformMatrix4fv(mvp_matrix_uniform, 1, GL_FALSE, mvp_matrix.data());
	glUniform3fv(cam_pos_uniform, 1, cam_pos.data());
	glUniformMatrix4fv(transposeInv_uniform, 1, GL_FALSE, transpose.data());
	glBindImageTexture(0, glModel->GetTSDFTex(), 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32F);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, depth_map_uniform);
	glActiveTexture(GL_TEXTURE1);
	glBindImageTexture(1, glModel->GetWeightTex(), 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32F);

	//todo loop over slide
	glDispatchCompute(resolutionX*resolutionY, resolutionX*resolutionY, 1);

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
