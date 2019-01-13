
#ifndef _RENDERER_H
#define _RENDERER_H

class GLModel;
class Window;
class CameraTransform;

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

		GLuint box_program = 0;
		GLint box_mvp_matrix_uniform = -1;

		GLuint fbo;
		GLuint color_tex;
		GLuint depth_tex;
		int fbo_width;
		int fbo_height;

		void InitResources();

	public:
		explicit Renderer(Window *window);
		~Renderer();

		void Render(GLModel *model, CameraTransform *camera_transform);
};

#endif //_RENDERER_H