
#ifndef _FRAME_H
#define _FRAME_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>

class Frame
{
	private:
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		float* depthMap;

	public:
		Frame();
		~Frame();

		int depthResolutionX;
		int depthResolutionY;

		pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud()	{ return cloud; }
		float* GetDepthMap() { return depthMap;  }
		void SetDepthMap(float* depthmap) { this->depthMap = depthMap; }
};

#endif //_FRAME_H
