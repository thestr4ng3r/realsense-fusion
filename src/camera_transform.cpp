
#include <camera_transform.h>

#include "camera_transform.h"

CameraTransform::CameraTransform()
{
	transform = Eigen::Affine3f::Identity();
}
