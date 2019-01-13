
#ifndef _CAMERA_TRANSFORM_H
#define _CAMERA_TRANSFORM_H

#include <Eigen/Geometry>

class CameraTransform
{
	private:
		Eigen::Affine3f transform;

	public:
		CameraTransform();

		const Eigen::Affine3f &GetTransform()		{ return transform; }
		void SetTransform(const Eigen::Affine3f &t)	{ transform = t; }

		Eigen::Matrix4f GetModelView()				{ return transform.inverse().matrix(); }
};

#endif //_CAMERA_TRANSFORM_H
