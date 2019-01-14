
#ifndef _FRAME_H
#define _FRAME_H

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <vector>

#include "window.h"

#include <Eigen/Core>

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

		Eigen::Vector2f intrinsics_focal_length;
		Eigen::Vector2f intrinsics_center;
		GLuint camera_intrinsics_buffer;

		GLuint process_program;
		GLint depth_scale_uniform;

	public:
		Frame();
		~Frame();

		//pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud()	{ return cloud; }
		//float* GetDepthMap() { return depthMap;  }
		//void SetDepthMap(float* depthmap) { this->depthMap = depthMap; }

		void SetDepthMap(int width, int height, GLushort *data, float depth_scale, const Eigen::Vector2f &focal_length, const Eigen::Vector2f &center);
		GLuint GetDepthTex()	{ return depth_tex; }
		int GetDepthWidth()		{ return depth_width; }
		int GetDepthHeight()	{ return depth_height; }
		float GetDepthScale()	{ return depth_scale; }

		GLuint GetVertexTex()	{ return vertex_tex; }
		GLuint GetNormalTex()	{ return normal_tex; }
		
		GLuint GetCameraIntrinsicsBuffer()	{ return camera_intrinsics_buffer; }

		void ProcessFrame();
};

#endif //_FRAME_H
