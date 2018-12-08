
#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

#include <stdio.h>


void points_to_pcl(const rs2::points& points, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

int main()
{
	try
	{
		rs2::pointcloud pc;
		rs2::points points;

		rs2::pipeline pipe;
		pipe.start();

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::visualization::CloudViewer cloud_viewer("Cloud Viewer");

		while(!cloud_viewer.wasStopped())
		{
			auto frames = pipe.wait_for_frames();
			auto depth = frames.get_depth_frame();
			points = pc.calculate(depth);

			/*auto color = frames.get_color_frame();
			// For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
			if (!color)
				color = frames.get_infrared_frame();
			pc.map_to(color);*/

			points_to_pcl(points, cloud);

			pcl::PassThrough<pcl::PointXYZ> z_filter;
			z_filter.setInputCloud(cloud);
			z_filter.setFilterFieldName("z");
			z_filter.setFilterLimits(-1.5f, 0.0f);
			z_filter.filter(*filtered_cloud);

			cloud_viewer.showCloud(filtered_cloud);
		}

		return 0;
	}
	catch (const rs2::error & e)
	{
		std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
		return 1;
	}
}


void points_to_pcl(const rs2::points& points, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
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
