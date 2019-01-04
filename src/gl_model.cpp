
#include "gl_model.h"


GLModel::GLModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize)
	: Model(resolutionX, resolutionY, resolutionZ, cellSize)
{
	Init();
}

GLModel::GLModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize, Eigen::Vector3f modelOrigin)
	: Model(resolutionX, resolutionY, resolutionZ, cellSize, modelOrigin)
{
	Init();
}

GLModel::~GLModel()
{

}

void GLModel::Init()
{
	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &tsdf_tex);
	glBindTexture(GL_TEXTURE_3D, tsdf_tex);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, resolutionX, resolutionY, resolutionZ, 0, GL_RED, GL_FLOAT, nullptr);

	glActiveTexture(GL_TEXTURE1);
	glGenTextures(1, &weight_tex);
	glBindTexture(GL_TEXTURE_3D, weight_tex);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, resolutionX, resolutionY, resolutionZ, 0, GL_RED, GL_FLOAT, nullptr);
}

void GLModel::CopyFrom(CPUModel *cpu_model)
{
	assert(cpu_model->GetResolutionX() == resolutionX);
	assert(cpu_model->GetResolutionY() == resolutionY);
	assert(cpu_model->GetResolutionZ() == resolutionZ);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_3D, tsdf_tex);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, resolutionX, resolutionY, resolutionZ, 0, GL_RED, GL_FLOAT, cpu_model->GetData());

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_3D, weight_tex);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, resolutionX, resolutionY, resolutionZ, 0, GL_RED, GL_FLOAT, cpu_model->GetWeights());

}