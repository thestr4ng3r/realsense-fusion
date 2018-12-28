
#include "window.h"
#include "renderer.h"
#include "model.h"

int main(int argc, char *argv[])
{
	auto model = new CPUModel(128, 128, 128, 1.0f / 128.0f);
	model->GenerateSphere(0.3f, Eigen::Vector3f(0.0f, 0.0f, 0.0f));

	Window window;
	Renderer renderer(&window);

	while(!window.GetShouldTerminate())
	{
		window.Update();
		window.BeginRender();
		renderer.UpdateModel(model);
		renderer.Render();
		window.EndRender();
	}

	delete model;
}