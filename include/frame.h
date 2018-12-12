
#ifndef _FRAME_H
#define _FRAME_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Frame
{
	private:
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

	public:
		Frame();
		~Frame();

		pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud()	{ return cloud; }
};

#endif //_FRAME_H
