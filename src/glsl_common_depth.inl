R"glsl(
float ReadDepth(usampler2D depth_tex, ivec2 pos, float depth_scale)
{
	uint depth_raw = texelFetch(depth_tex, pos, 0).r;
	return float(depth_raw) * depth_scale;
}
)glsl"