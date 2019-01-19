
#ifdef ENABLE_INPUT_REALSENSE
#ifndef _REALSENSE_INPUT_H
#define _REALSENSE_INPUT_H

#include <librealsense2/rs.hpp>

#include "input.h"

class RealSenseInput : public Input
{
	private:
		rs2::pipeline pipe;
		//rs2::pointcloud pc;
		//rs2::points points;

		rs2_intrinsics intrinsics;
		rs2_intrinsics IntrinsicsColor;
		float depth_scale;

		bool filters_active = false;
		bool color_active = false;

	public:
		RealSenseInput(const rs2::config &config = rs2::config());
		~RealSenseInput();

		bool WaitForFrame(Frame *frame) override;

		float GetPpx() { return intrinsics.ppx; }
		float GetPpy() { return intrinsics.ppy; }
		float GetFx() { return intrinsics.fx; }
		float GetFy() { return intrinsics.fy; }

		float GetPpxColor() { return IntrinsicsColor.ppx; }
		float GetPpyColor() { return IntrinsicsColor.ppy; }
		float GetFxColor() { return IntrinsicsColor.fx; }
		float GetFyColor() { return IntrinsicsColor.fy; }

		void setFilterActive(bool set) { filters_active = set; }
		void setColorActive(bool set) { color_active = set; }
};

#endif //INPUT_H
#endif