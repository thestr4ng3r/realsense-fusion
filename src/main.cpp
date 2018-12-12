
#include "realsense_input.h"
#include "frame.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>

#include <stdio.h>

int main(int argc, char *argv[])
{
	Input *input;

	input = new RealSenseInput();

	Frame frame;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

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
	}

	return 0;
}
