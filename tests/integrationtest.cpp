#include "input.h"

#ifdef ENABLE_INPUT_REALSENSE
#include "realsense_input.h"
#endif

#include "frame.h"

#include "window.h"
#include "renderer.h"
#include "gl_model.h"
#include "pc_integrator.h"

int main(int argc, char *argv[])
{
	Input *input;
#if defined(ENABLE_INPUT_REALSENSE)
	input = new RealSenseInput();
#if defined(ENABLE_INPUT_KINECT)
	//#warning "Building with both RealSense and Kinect. Using RealSense."
#endif
#elif defined(ENABLE_INPUT_KINECT)
	// TODO: input = ... for kinect
#else
#error "Build with no input!"
#endif

	Window window("Integration Test");
	Frame frame;

	bool integrated = false;
	
	/*
	CPUModel cpu_model(512, 512, 512, 1.0f/512.f);
	//cpu_model.GenerateSphere(0.25, Eigen::Vector3f(0.0, 0.0, 0.0));
	cpu_model.DebugToLog();
	GLModel glmodel(512, 512, 512, 1.0f / 512.f);
	glmodel.CopyFrom(&cpu_model);
	*/

	CPUModel cpu_model (128, 128, 128, 1.0f / 128.0f);
	cpu_model.GenerateSphere(0.3f, Eigen::Vector3f(0.0f, 0.0f, 0.0f));

	GLModel glmodel(
		cpu_model.GetResolutionX(),
		cpu_model.GetResolutionY(),
		cpu_model.GetResolutionZ(),
		cpu_model.GetCellSize(),
		cpu_model.GetModelOrigin());

	glmodel.CopyFrom(&cpu_model);

	PC_Integrator integrator (&glmodel, input);

	Renderer renderer(&window);

	while (!window.GetShouldTerminate())
	{
		window.Update();

		if (!input->WaitForFrame(&frame))
		{
			std::cerr << "Failed to get frame." << std::endl;
			continue;
		}

		/*if (!integrated)
		{*/
			integrator.integrate(&frame);
			integrated = true;
		//}

		window.BeginRender();
		renderer.Render(&glmodel);
		window.EndRender();
	}

	delete input;
	return 0;

}
