
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

	GLModel gl_model(256, 256, 256, 4.0f / 256.f, 0.3f, -0.3f, true);

	Renderer renderer(&window);

	CameraTransform camera_transform;

	Eigen::Affine3f reset_transform = Eigen::Affine3f::Identity();
	reset_transform.translate(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
	camera_transform.SetTransform(reset_transform);

	ICP icp;

	PC_Integrator integrator(&gl_model);

	bool enable_perf_measure = false;
	bool enable_tracking = true;
	int icp_passes = 5;

	bool render_color = false;
	bool render_lighting = true;

	bool show_tex_input_normal = false;
#ifdef ICP_DEBUG_TEX
	bool show_tex_icp_debug = false;
#endif
	bool show_tex_renderer_vertex = false;
	bool show_tex_renderer_normal = false;

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
		renderer.SetEnableColor(render_color);
		renderer.SetEnableLighting(render_lighting);
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

		if(ImGui::TreeNode("Rendering"))
		{
			ImGui::Checkbox("Enable Color", &render_color);
			ImGui::Checkbox("Enable Lighting", &render_lighting);
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

		if(ImGui::TreeNode("Debug Textures"))
		{
			ImGui::Checkbox("Input Normal", &show_tex_input_normal);
#ifdef ICP_DEBUG_TEX
			ImGui::Checkbox("ICP Debug", &show_tex_icp_debug);
#endif
			ImGui::Checkbox("Renderer Vertex", &show_tex_renderer_vertex);
			ImGui::Checkbox("Renderer Normal", &show_tex_renderer_normal);
			ImGui::TreePop();
		}

		ImGui::End();

		auto ShowTexture = [&frame](const char *title, GLuint tex, bool flip = false)
		{
			ImVec2 tex_size(frame.GetDepthWidth(), frame.GetDepthHeight());
			ImGui::Begin(title, nullptr, ImVec2(tex_size.x * 0.3f, tex_size.y * 0.3f), 1.0f);
			ImGui::Image(reinterpret_cast<ImTextureID>(tex), ImGui::GetContentRegionAvail(),
					ImVec2(0.0f, flip ? 1.f : 0.0f), ImVec2(1.0f, flip ? 0.0f : 1.0f), ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
			ImGui::End();
		};

		if(show_tex_input_normal)
			ShowTexture("Input Normal", frame.GetNormalTex());
#ifdef ICP_DEBUG_TEX
		if(show_tex_icp_debug)
			ShowTexture("ICP Debug", icp.GetDebugTex());
#endif
		if(show_tex_renderer_vertex)
			ShowTexture("Renderer Vertex", renderer.GetVertexTex(), true);
		if(show_tex_renderer_normal)
			ShowTexture("Renderer Normal", renderer.GetNormalTex(), true);

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
