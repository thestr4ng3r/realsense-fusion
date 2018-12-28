
#ifndef _WINDOW_H
#define _WINDOW_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>

class Window
{
	private:
		GLFWwindow *window = nullptr;
		bool should_terminate;

	public:
		Window();
		~Window();

		void GetSize(int *width, int *height)	{ glfwGetFramebufferSize(window, width, height); }
		bool GetShouldTerminate()				{ return should_terminate; }

		void Update();
		void BeginRender();
		void EndRender();
};

#endif //_WINDOW_H
