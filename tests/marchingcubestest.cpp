#include "model.h"
#include "marching_cubes.h"
#include <stdio.h>

int main(int argc, char *argv[])
{
	std::cout << "Marching Cubes Test \n";

	CPUModel model(64, 64, 64, 1.0f / 64.0f, 0.3f, -0.3f, false);
	model.GenerateSphere(0.3f, Eigen::Vector3f(0.0f, 0.0f, 0.0f));

	Marching_Cubes mc(&model);
	mc.process_mc();	

	return 0;
}