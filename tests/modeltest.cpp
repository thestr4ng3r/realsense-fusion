#include "model.h"
#include <stdio.h>

int main(int argc, char *argv[])
{
	std::cout << "Model Test \n";

	Model model(4, 4, 4, 0.1);

	model.debugToLog();

	system("pause");
	return 0;
}