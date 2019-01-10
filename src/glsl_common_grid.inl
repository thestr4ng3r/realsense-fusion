R"glsl(
layout(std140, binding=0) uniform GridParamsBlock
{
	uvec3 res;
	float cell_size;
	vec3 origin;
} grid_params;

// pos is in [0.0, 1.0]
// returns the absolute world position
vec3 GridToWorld(vec3 pos)
{
	return pos * vec3(grid_params.res) * grid_params.cell_size + grid_params.origin;
}

// returns the position in the grid in [0.0, 1.0]
vec3 WorldToGrid(vec3 pos)
{
	return ((pos - grid_params.origin) / grid_params.cell_size) / vec3(grid_params.res);
}

// pos is in [0.0, 1.0]
// returns the texel position in [0, grid_params.res]
ivec3 GridToTexel(vec3 pos)
{
	return ivec3(vec3(grid_params.res) * pos);
}

vec3 GridExtent()
{
	return vec3(grid_params.res) * grid_params.cell_size;
}
)glsl"