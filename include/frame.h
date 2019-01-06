
#ifndef _FRAME_H
#define _FRAME_H

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <vector>

#include "window.h"

class Frame
{
	private:
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		GLuint depth_tex;
		GLuint vertex_tex;
		GLuint normal_tex;

		int depth_width;
		int depth_height;
		float depth_scale;

		GLuint process_program;
		GLint depth_scale_uniform;

	public:
		Frame();
		~Frame();

		//pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud()	{ return cloud; }
		//float* GetDepthMap() { return depthMap;  }
		//void SetDepthMap(float* depthmap) { this->depthMap = depthMap; }

		void SetDepthMap(int width, int height, GLushort *data, float depth_scale);
		GLuint GetDepthTex()	{ return depth_tex; }
		int GetDepthWidth()		{ return depth_width; }
		int GetDepthHeight()	{ return depth_height; }

		void ProcessFrame();
};

#endif //_FRAME_H
