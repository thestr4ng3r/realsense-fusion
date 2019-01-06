#include "input.h"

#ifdef ENABLE_INPUT_REALSENSE
#include "realsense_input.h"
#endif

#include "frame.h"

#include "window.h"
//#include "renderer.h"
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

	Frame frame;
	Window window;

	while (!window.GetShouldTerminate())
	{
		window.Update();
		window.BeginRender();
		window.EndRender();

		if (!input->WaitForFrame(&frame))
		{
			std::cerr << "Failed to get frame." << std::endl;
			continue;
		}
		else {
			break; //first frame ready for computing
		}

		GLModel glmodel (1024, 1024, 1024, 0.1);
		glmodel.CopyFrom(&CPUModel(1024, 1024, 1024, 0.1));

		PC_Integrator integrator (&glmodel);
		integrator.integrate(&frame);

	}

	delete input;
	return 0;

}
