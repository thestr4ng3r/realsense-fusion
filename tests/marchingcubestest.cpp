#include "model.h"
#include "marching_cubes.h"
#include <stdio.h>

int main(int argc, char *argv[])
{
	std::cout << "Marching Cubes Test \n";

	CPUModel model(64, 64, 64, 1.0f / 64.0f);
	model.GenerateSphere(0.3f, Eigen::Vector3f(0.0f, 0.0f, 0.0f));

	float* tdsf = model.GetData();

	Marching_Cubes mc(&model);
	mc.process_mc();	

	return 0;
}