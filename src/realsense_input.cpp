
#ifdef ENABLE_INPUT_REALSENSE
#include "frame.h"
#include "realsense_input.h"

RealSenseInput::RealSenseInput()
{
	try
	{
		pipe.start();
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


void PointsToPCL(const rs2::points& points, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

bool RealSenseInput::WaitForFrame(Frame *frame)
{
	try
	{
		auto frames = pipe.wait_for_frames();
		auto depth = frames.get_depth_frame();
		points = pc.calculate(depth);
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

	auto cloud = frame->GetCloud();
	PointsToPCL(points, cloud);

	return true;
}

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
}

#endif