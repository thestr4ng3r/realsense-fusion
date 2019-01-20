
#include "window.h"
#include "renderer.h"
#include "gl_model.h"

int main(int argc, char *argv[])
{
	Window window("new",1280,640);
	Renderer renderer(&window);
	
	auto cpu_model = new CPUModel(128, 128, 128, 1.0f / 128.0f,false);
	cpu_model->GenerateSphere(0.3f, Eigen::Vector3f(0.0f, 0.0f, 0.0f));

	auto gl_model = new GLModel(
			cpu_model->GetResolutionX(),
			cpu_model->GetResolutionY(),
			cpu_model->GetResolutionZ(),
			cpu_model->GetCellSize(),
			cpu_model->GetModelOrigin(),
			false);

	while(!window.GetShouldTerminate())
	{
		window.Update();

		gl_model->CopyFrom(cpu_model);

		window.BeginRender();
		renderer.Render(gl_model);
		window.EndRender();
	}

	delete cpu_model;
	delete gl_model;
}