
#include "frame.h"

Frame::Frame() :
	cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
}

Frame::~Frame()
{
}

