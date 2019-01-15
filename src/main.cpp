
#include "input.h"

#ifdef ENABLE_INPUT_REALSENSE
#include "realsense_input.h"
#endif

#include "frame.h"

#include "window.h"
#include "gl_model.h"
#include "renderer.h"
#include "camera_transform.h"
#include "pc_integrator.h"
#include "icp.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

#include "imgui.h"

int main(int argc, char *argv[])
{
	Window window("Scanner", 1280, 720);

	Input *input;

#if defined(ENABLE_INPUT_REALSENSE)
	rs2::config rs_config;
	if(argc >= 2)
		rs_config.enable_device_from_file(std::string(argv[1]));
	input = new RealSenseInput(rs_config);
#if defined(ENABLE_INPUT_KINECT)
//#warning "Building with both RealSense and Kinect. Using RealSense."
#endif
#elif defined(ENABLE_INPUT_KINECT)
	// TODO: input = ... for kinect
#else
#error "Build with no input!"
#endif

	Frame frame;

	GLModel gl_model(128, 128, 128, 1.0f / 128.0f);
	{
		CPUModel test_model(
				gl_model.GetResolutionX(),
				gl_model.GetResolutionY(),
				gl_model.GetResolutionZ(),
				gl_model.GetCellSize());
		test_model.GenerateSphere(0.2f, Eigen::Vector3f(0.0f, 0.0f, 0.0f));
		gl_model.CopyFrom(&test_model);
	}

	Renderer renderer(&window);

	CameraTransform camera_transform;

	Eigen::Affine3f t = Eigen::Affine3f::Identity();
	t.translate(Eigen::Vector3f(0.0f, 0.0f, 1.5f));
	camera_transform.SetTransform(t);

	ICP icp;

	PC_Integrator integrator(&gl_model);

	while(!window.GetShouldTerminate())
	{
		window.Update();

		if(!input->WaitForFrame(&frame))
		{
			std::cerr << "Failed to get frame." << std::endl;
			continue;
		}
		frame.ProcessFrame();

		CameraTransform camera_transform_old = camera_transform;
		icp.SearchCorrespondences(&frame, &renderer, camera_transform_old, &camera_transform);

		integrator.integrate(&frame, &camera_transform);

		window.BeginGUI();
		ImGui::Begin("Info");
		ImGui::Text("Resolution: %dx%d", frame.GetDepthWidth(), frame.GetDepthHeight());
		ImGui::End();
		window.EndGUI();

		window.BeginRender();
		renderer.Render(&gl_model, &frame, &camera_transform);
		window.EndRender();
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
