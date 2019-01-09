
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "window.h"

#include <stdexcept>
#include <window.h>

// https://www.khronos.org/opengl/wiki/Debug_Output#Examples
void GLAPIENTRY GLMessageCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void *user)
{
	const char *type_str;
	switch(type)
	{
		case GL_DEBUG_TYPE_ERROR:
			type_str = "Error";
			break;
		case GL_DEBUG_TYPE_PERFORMANCE:
			type_str = "Perf";
			break;
		case GL_DEBUG_TYPE_OTHER:
			type_str = "Other";
			break;
		default:
			type_str = "???";
			break;
	}
	fprintf(stderr, "GL: %s type = %#x, severity = %#x, message = %s\n", type_str, type, severity, message);
}

Window::Window(const char *title)
{
	if(!glfwInit())
		throw std::runtime_error("Failed to initialize GLFW.");

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	window = glfwCreateWindow(640, 480, title, nullptr, nullptr);

	if(!window)
	{
		glfwTerminate();
		throw std::runtime_error("Failed to create window.");
	}

	glfwMakeContextCurrent(window);
	glewInit();

	glEnable(GL_DEBUG_OUTPUT);
	glDebugMessageCallback(GLMessageCallback, nullptr);

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO &imgui_io = ImGui::GetIO();

	ImGui::StyleColorsDark();

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(GLSL_VERSION);


	should_terminate = false;
}

Window::~Window()
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
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
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	glfwSwapBuffers(window);
}

void Window::BeginGUI()
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

void Window::EndGUI()
{
	ImGui::Render();
}

