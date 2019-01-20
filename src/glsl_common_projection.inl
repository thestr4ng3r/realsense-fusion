R"glsl(
layout(std140, binding=1) uniform CameraIntrinsicsBlock
{
	vec2 focal_length;
	vec2 center;
	uvec2 res;
} camera_intrinsics;

layout(std140, binding=2) uniform CameraColorIntrinsicsBlock
{
	vec2 focal_length;
	vec2 center;
	uvec2 res;
} camera_color_intrinsics;

// see rs2_project_point_to_pixel()
// returns a pixel coordinate in [0, camera_intrinsics.res]
vec2 ProjectCameraToImage(vec3 pos)
{
	vec2 v = (pos.xy * vec2(1.0, -1.0)) / -pos.z;
	// TODO: distortion here
	v *= camera_intrinsics.focal_length;
	v += camera_intrinsics.center;
	return v;
}

// same as ProjectCameraToImage just on Color Image
// returns a pixel coordinate in [0, camera_intrinsics.res]
vec2 ProjectColorCameraToImage(vec3 pos)
{
	vec2 v = (pos.xy * vec2(1.0, -1.0)) / -pos.z;
	// TODO: distortion here
	v *= camera_color_intrinsics.focal_length;
	v += camera_color_intrinsics.center;
	return v;
}

// see rs2_deproject_pixel_to_point
// pos is a pixel coordinate in [0, camera_intrinsics.res]
vec3 DeprojectImageToCamera(vec2 pos, float depth)
{
	vec2 v = pos - camera_intrinsics.center;
	v /= camera_intrinsics.focal_length;
	// TODO: distortion here
	return vec3(v, -1.0) * depth * vec3(1.0, -1.0, 1.0);
}

// same as DeprojectImageToCamera just in From Color Image
// pos is a pixel coordinate in [0, camera_intrinsics.res]
vec3 DeprojectColorImageToCamera(vec2 pos, float depth)
{
	vec2 v = pos - camera_color_intrinsics.center;
	v /= camera_color_intrinsics.focal_length;
	// TODO: distortion here
	return vec3(v, -1.0) * depth * vec3(1.0, -1.0, 1.0);
}

)glsl"