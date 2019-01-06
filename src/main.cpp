
#include "input.h"

#ifdef ENABLE_INPUT_REALSENSE
#include "realsense_input.h"
#endif

#include "frame.h"

#include "window.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char *argv[])
{
	Window window;

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

	while(!window.GetShouldTerminate())
	{
		window.Update();
		window.BeginRender();
		window.EndRender();

		if(!input->WaitForFrame(&frame))
		{
			std::cerr << "Failed to get frame." << std::endl;
			continue;
		}
		frame.ProcessFrame();
	}

	delete input;

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::visualization::CloudViewer cloud_viewer("Cloud Viewer");

	while(!cloud_viewer.wasStopped())
	{
		input->WaitForFrame(&frame);

		pcl::PassThrough<pcl::PointXYZ> z_filter;
		z_filter.setInputCloud(frame.GetCloud());
		z_filter.setFilterFieldName("z");
		z_filter.setFilterLimits(-1.5f, 0.0f);
		z_filter.filter(*filtered_cloud);

		cloud_viewer.showCloud(filtered_cloud);
	}*/

	return 0;
}
