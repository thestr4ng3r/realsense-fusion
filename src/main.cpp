
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

//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/passthrough.h>

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

	GLModel gl_model(128, 128, 128, 1.0f / 128.0f, 0.3f, -0.3f, false);

	Renderer renderer(&window);

	CameraTransform camera_transform;

	Eigen::Affine3f reset_transform = Eigen::Affine3f::Identity();
	reset_transform.translate(Eigen::Vector3f(0.0f, 0.0f, 0.7f));
	camera_transform.SetTransform(reset_transform);

	ICP icp;

	PC_Integrator integrator(&gl_model);

	bool enable_tracking = true;
	int icp_passes = 5;

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
		for(int i=0; i<icp_passes; i++)
		{
			icp.SearchCorrespondences(&frame, &renderer, camera_transform_old, &camera_transform);
			icp.SolveMatrix(&camera_transform);
		}
		if(!enable_tracking)
			camera_transform = camera_transform_old;

		integrator.integrate(&frame, &camera_transform);

		window.BeginGUI();
		ImGui::Begin("Settings");
		ImGui::Text("Resolution: %dx%d", frame.GetDepthWidth(), frame.GetDepthHeight());
		if(ImGui::Button("Reset Model and Transform"))
		{
			gl_model.Reset();
			camera_transform.SetTransform(reset_transform);
		}
		ImGui::BeginChild("ICP", ImVec2(0, 0), true);
			ImGui::Checkbox("Enable Tracking", &enable_tracking);
			ImGui::SliderInt("Iterations", &icp_passes, 1, 10);
			float v = icp.GetDistanceThreshold();
			ImGui::SliderFloat("Distance Threshold", &v, 0.0f, 1.0f, "%.3f");
			icp.SetDistanceThreshold(v);
			v = icp.GetAngleThreshold();
			ImGui::SliderFloat("Angle Threshold", &v, -1.0f, 1.0f, "%.3f");
			icp.SetAngleThreshold(v);
		ImGui::EndChild();
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
