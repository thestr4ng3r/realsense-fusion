
#include "renderer.h"
#include "model.h"

int main(int argc, char *argv[])
{
	Model *model = new Model(128, 128, 128, 1.0f / 128.0f);
	model->GenerateSphere(0.3f, Eigen::Vector3f(0.0f, 0.0f, 0.0f));

	Renderer renderer;

	while(!renderer.GetShouldTerminate())
	{
		renderer.UpdateModel(model);
		renderer.Update();
	}

	delete model;
}