
#ifdef ENABLE_INPUT_REALSENSE
#include "frame.h"
#include "realsense_input.h"

#include <iostream>

RealSenseInput::RealSenseInput()
{
	try
	{
		pipe.start();
		intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
	}
	catch(const rs2::error &e)
	{
		std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
		throw std::exception();
	}
}

RealSenseInput::~RealSenseInput()
{
	pipe.stop();
}


bool RealSenseInput::WaitForFrame(Frame *frame)
{
	try
	{
		auto frames = pipe.wait_for_frames();
		auto depth = frames.get_depth_frame();
		if (depth.get_profile().stream_type() != RS2_STREAM_DEPTH || depth.get_profile().format() != RS2_FORMAT_Z16)
		{
			std::cerr << "Frame from RealSense has invalid stream type or format." << std::endl;
			return false;
		}
		frame->SetDepthMap(depth.get_width(), depth.get_height(), (GLushort *)depth.get_data());
	}
	catch(const rs2::error &e)
	{
		std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
		return false;
	}

	/*auto color = frames.get_color_frame();
	// For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
	if (!color)
		color = frames.get_infrared_frame();
	pc.map_to(color);*/

	//auto cloud = frame->GetCloud();
	//PointsToPCL(points, cloud);


	return true;
}


/*void PointsToPCL(const rs2::points& points, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
void PointsToPCL(const rs2::points& points, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = -ptr->y;
		p.z = -ptr->z;
		ptr++;
	}
}*/

#endif