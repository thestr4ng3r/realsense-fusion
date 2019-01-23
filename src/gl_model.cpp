
#include "gl_model.h"


GLModel::GLModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize, float max_truncation, float min_truncation, bool colorsActive)
	: Model(resolutionX, resolutionY, resolutionZ, cellSize, max_truncation, min_truncation, colorsActive)
{
	this->colorsActive = colorsActive;
	Init();
}

GLModel::GLModel(int resolutionX, int resolutionY, int resolutionZ, float cellSize, float max_truncation, float min_truncation, Eigen::Vector3f modelOrigin, bool colorsActive)
	: Model(resolutionX, resolutionY, resolutionZ, cellSize, max_truncation, min_truncation, modelOrigin, colorsActive)
{
	this->colorsActive = colorsActive;
	Init();
}

GLModel::~GLModel()
{
	glDeleteTextures(1, &tsdf_tex);
	glDeleteTextures(1, &weight_tex);
	glDeleteBuffers(1, &params_buffer);
	if (colorsActive) {
		glDeleteTextures(1, &color_tex);
	}
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
	glTexImage3D(GL_TEXTURE_3D, 0, GL_R16UI, resolutionX, resolutionY, resolutionZ, 0, GL_RED_INTEGER, GL_UNSIGNED_SHORT, nullptr);

	if (colorsActive) {
		glActiveTexture(GL_TEXTURE2);
		glGenTextures(1, &color_tex);
		glBindTexture(GL_TEXTURE_3D, color_tex);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
		glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA, resolutionX, resolutionY, resolutionZ, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
	}

	// see glsl_common_grid.inl
	glGenBuffers(1, &params_buffer);
	glBindBuffer(GL_UNIFORM_BUFFER, params_buffer);
	uint32_t buf[8];
	buf[0] = static_cast<uint32_t>(resolutionX);
	buf[1] = static_cast<uint32_t>(resolutionY);
	buf[2] = static_cast<uint32_t>(resolutionZ);
	*((float *)(buf + 3)) = cellSize;
	*((float *)(buf + 4)) = modelOrigin.x();
	*((float *)(buf + 5)) = modelOrigin.y();
	*((float *)(buf + 6)) = modelOrigin.z();
	buf[7] = 0;
	glBufferData(GL_UNIFORM_BUFFER, sizeof(buf), buf, GL_STATIC_DRAW);

	Reset();
}

void GLModel::Reset()
{
	float tsdf_reset[] = { max_truncation, 0.0f, 0.0f, 0.0f };
	glClearTexImage(tsdf_tex, 0, GL_RGBA, GL_FLOAT, tsdf_reset);
	uint16_t weight_reset[] = { 0, 0, 0, 0 };
	glClearTexImage(weight_tex, 0, GL_RGBA_INTEGER, GL_UNSIGNED_SHORT, weight_reset);
	if (colorsActive)
	{
		uint8_t color_reset[] = { 0, 0, 0, 0 };
		glClearTexImage(color_tex, 0, GL_RGBA, GL_UNSIGNED_BYTE, color_reset);
	}
}

void GLModel::CopyFrom(CPUModel *cpu_model)
{
	assert(cpu_model->GetResolutionX() == resolutionX);
	assert(cpu_model->GetResolutionY() == resolutionY);
	assert(cpu_model->GetResolutionZ() == resolutionZ);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_3D, tsdf_tex);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, resolutionX, resolutionY, resolutionZ, 0, GL_RED, GL_FLOAT, cpu_model->GetData());

	glBindTexture(GL_TEXTURE_3D, weight_tex);
	glTexImage3D(GL_TEXTURE_3D, 0, GL_R16UI, resolutionX, resolutionY, resolutionZ, 0, GL_RED_INTEGER, GL_UNSIGNED_SHORT, cpu_model->GetWeights());

	if (colorsActive)
	{
		glBindTexture(GL_TEXTURE_3D, color_tex);
		glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA, resolutionX, resolutionY, resolutionZ, 0, GL_RGBA, GL_UNSIGNED_BYTE, cpu_model->GetColor());
	}
}

void GLModel::CopyTo(CPUModel *cpu_model)
{
	assert(cpu_model->GetResolutionX() == resolutionX);
	assert(cpu_model->GetResolutionY() == resolutionY);
	assert(cpu_model->GetResolutionZ() == resolutionZ);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_3D, tsdf_tex);
	glGetTexImage(GL_TEXTURE_3D, 0, GL_RED, GL_FLOAT, cpu_model->GetData());

	glBindTexture(GL_TEXTURE_3D, weight_tex);
	glGetTexImage(GL_TEXTURE_3D, 0, GL_RED_INTEGER, GL_UNSIGNED_SHORT, cpu_model->GetWeights());

	if (colorsActive && cpu_model->GetColorsActive())
	{
		glBindTexture(GL_TEXTURE_3D, color_tex);
		glGetTexImage(GL_TEXTURE_3D, 0, GL_RGBA, GL_UNSIGNED_BYTE, cpu_model->GetColor());
	}
}