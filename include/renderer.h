
#ifndef _RENDERER_H
#define _RENDERER_H

class CPUModel;
class Window;

class Renderer
{
	private:
		Window *window;

		GLuint vbo = 0;
		GLuint vao = 0;
		GLuint ibo = 0;
		GLuint program = 0;

		GLint mvp_matrix_uniform = -1;
		GLint cam_pos_uniform = -1;

		GLint tsdf_tex_uniform = -1;

		GLuint tsdf_tex = 0;

		void InitResources();

	public:
		explicit Renderer(Window *window);
		~Renderer();

		void UpdateModel(CPUModel *model);
		void Render();
};

#endif //_RENDERER_H