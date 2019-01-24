
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
#include "marching_cubes.h"
#include <chrono>

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
	input->setColorActive(true);
#if defined(ENABLE_INPUT_KINECT)
//#warning "Building with both RealSense and Kinect. Using RealSense."
#endif
#elif defined(ENABLE_INPUT_KINECT)
	// TODO: input = ... for kinect
#else
#error "Build with no input!"
#endif

	Frame frame;

	GLModel gl_model(256, 256, 256, 2.0f / 256.f, 0.3f, -0.3f, true);

	Renderer renderer(&window, true);

	CameraTransform camera_transform;

	Eigen::Affine3f reset_transform = Eigen::Affine3f::Identity();
	reset_transform.translate(Eigen::Vector3f(0.0f, 0.0f, 0.7f));
	camera_transform.SetTransform(reset_transform);

	ICP icp;

	PC_Integrator integrator(&gl_model);

	bool enable_perf_measure = false;
	bool enable_tracking = true;
	int icp_passes = 5;

	using clock = std::chrono::steady_clock;
	using time_point = std::chrono::time_point<clock>;
	time_point time_begin;
	time_point time_process_frame;
	time_point time_icp;
	std::vector<time_point> time_icp_corr;
	std::vector<time_point> time_icp_solve;
	time_point time_integrate;
	time_point time_render;
	auto MeasureTime = [&enable_perf_measure](time_point &tp) {
		if(!enable_perf_measure)
			return;
		glFinish();
		tp = clock::now();
	};


	while(!window.GetShouldTerminate())
	{
		window.Update();

		if(!input->WaitForFrame(&frame))
		{
			std::cerr << "Failed to get frame." << std::endl;
			continue;
		}

		MeasureTime(time_begin);

		frame.ProcessFrame();

		MeasureTime(time_process_frame);

		if (enable_tracking)
		{
			if(enable_perf_measure)
			{
				time_icp_corr.resize(static_cast<unsigned long>(icp_passes));
				time_icp_solve.resize(static_cast<unsigned long>(icp_passes));
			}
			for(int i=0; i<icp_passes; i++)
			{
				icp.SearchCorrespondences(&frame, &renderer, camera_transform);
				if(enable_perf_measure)
				{
					glFinish();
					time_icp_corr[i] = clock::now();
				}
				icp.SolveMatrix(&camera_transform);
				if(enable_perf_measure)
				{
					glFinish();
					time_icp_solve[i] = clock::now();
				}
			}
		}

		MeasureTime(time_icp);

		integrator.integrate(&frame, &camera_transform);

		MeasureTime(time_integrate);

		window.BeginRender();
		renderer.Render(&gl_model, &frame, &camera_transform);

		MeasureTime(time_render);


		window.BeginGUI();
		ImGui::Begin("Settings");
		ImGui::Text("Resolution: %dx%d", frame.GetDepthWidth(), frame.GetDepthHeight());
		if(ImGui::Button("Reset Model and Transform"))
		{
			gl_model.Reset();
			camera_transform.SetTransform(reset_transform);
		}
		if(ImGui::Button("Export Mesh"))
		{
			CPUModel *cpu_model = new CPUModel(
					gl_model.GetResolutionX(), 
					gl_model.GetResolutionY(),
					gl_model.GetResolutionZ(),
					gl_model.GetCellSize(),
					gl_model.GetMaxTruncation(),
					gl_model.GetMinTruncation(),
					gl_model.GetModelOrigin(),
					gl_model.GetColorsActive());
			gl_model.CopyTo(cpu_model);
			
			// export the mesh with marching cubes
			Marching_Cubes mc(cpu_model);
			mc.process_mc("/home/florian/mesh.off");
			delete cpu_model;
		}

		if(ImGui::TreeNode("ICP"))
		{
			ImGui::Checkbox("Enable Tracking", &enable_tracking);
			ImGui::SliderInt("Iterations", &icp_passes, 1, 10);
			float v = icp.GetDistanceThreshold();
			ImGui::SliderFloat("Distance Threshold", &v, 0.0f, 1.0f, "%.3f");
			icp.SetDistanceThreshold(v);
			v = icp.GetAngleThreshold();
			ImGui::SliderFloat("Angle Threshold", &v, -1.0f, 1.0f, "%.3f");
			icp.SetAngleThreshold(v);
			ImGui::TreePop();
		}

		if(ImGui::TreeNode("Performance"))
		{
			bool enable_perf_measure_new = enable_perf_measure;
			ImGui::Checkbox("Measure Performance", &enable_perf_measure_new);
			if(enable_perf_measure)
			{
				auto RenderDuration = [](const char *title, const time_point &from, const time_point &to) {
					ImGui::Text("%s", title);
					ImGui::SameLine();
					std::chrono::duration<float, std::milli> duration = to - from;
					ImGui::Text("%f", duration.count());
				};

				RenderDuration("Process Frame", time_begin, time_process_frame);
				RenderDuration("ICP (total)", time_process_frame, time_icp);
				for(int i=0; i<icp_passes; i++)
				{
					RenderDuration(("  pass " + std::to_string(i) + " corr").c_str(), i > 0 ? time_icp_solve[i-1] : time_process_frame, time_icp_corr[i]);
					RenderDuration(("  pass " + std::to_string(i) + " solve").c_str(), time_icp_corr[i], time_icp_solve[i]);
				}
				RenderDuration("Integrate", time_icp, time_integrate);
				RenderDuration("Render", time_integrate, time_render);
			}
			enable_perf_measure = enable_perf_measure_new;
			ImGui::TreePop();
		}

		ImGui::End();

#ifdef ICP_DEBUG_TEX
		ImGui::Begin("ICP Debug");
		ImGui::Image(reinterpret_cast<ImTextureID>(icp.GetDebugTex()), ImVec2((float)icp.GetDebugTexWidth() * 0.3f, (float)icp.GetDebugTexHeight() * 0.3f));
		ImGui::End();
#endif

		window.EndGUI();

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
