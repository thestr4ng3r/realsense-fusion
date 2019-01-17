#ifndef _MARCHING_CUBES_H
#define _MARCHING_CUBES_H

#include "model.h"
#include "mesh.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

class Marching_Cubes {
public:
	Marching_Cubes();
	~Marching_Cubes();

	struct MC_Triangle;
	struct MC_Gridcell;
	struct MC_Gridcell_2;

	void process_mc(CPUModel* model);
	bool ProcessVolumeCell(CPUModel* model, int x, int y, int z, double iso, Mesh* mesh);
	int Polygonise(MC_Gridcell grid, double isolevel, MC_Triangle* triangles);
	Eigen::Vector3f VertexInterp(double isolevel, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, double valp1, double valp2);

};

#endif // MARCHING_CUBES_H