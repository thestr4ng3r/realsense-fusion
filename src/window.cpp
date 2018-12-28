
#include "window.h"

#include <stdexcept>
#include <window.h>


Window::Window()
{
	if(!glfwInit())
		throw std::runtime_error("Failed to initialize GLFW.");

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	window = glfwCreateWindow(640, 480, "Renderer", nullptr, nullptr);

	if(!window)
	{
		glfwTerminate();
		throw std::runtime_error("Failed to create window.");
	}

	glfwMakeContextCurrent(window);
	glewInit();

	should_terminate = false;
}

Window::~Window()
{
	glfwDestroyWindow(window);
	glfwTerminate();
}

void Window::Update()
{
	glfwPollEvents();
	should_terminate = static_cast<bool>(glfwWindowShouldClose(window));
}

void Window::BeginRender()
{
	int width, height;
	GetSize(&width, &height);

	glViewport(0, 0, width, height);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
}

void Window::EndRender()
{
	glfwSwapBuffers(window);
}
