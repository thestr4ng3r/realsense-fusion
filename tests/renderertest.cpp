
#include "renderer.h"

int main(int argc, char *argv[])
{
	Renderer renderer;

	while(!renderer.GetShouldTerminate())
	{
		renderer.Update();
	}
}