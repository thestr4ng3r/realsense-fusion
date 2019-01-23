
#ifndef _RENDERER_H
#define _RENDERER_H

#include <Eigen/Core>

class GLModel;
class Window;
class CameraTransform;
class Frame;

class Renderer
{
	private:
		Window *window;

		GLuint vbo = 0;
		GLuint vao = 0;
		GLuint ibo = 0;

		GLuint program = 0;
		GLint mvp_matrix_uniform = -1;
		GLint modelview_matrix_uniform = -1;
		GLint cam_pos_uniform = -1;
		GLint tsdf_tex_uniform = -1;
		GLint color_grid_tex_uniform = -1;
		GLint activate_colors_uniform = 0;

		GLuint box_program = 0;
		GLint box_mvp_matrix_uniform = -1;

		GLuint fbo;
		GLuint color_tex;
		GLuint vertex_tex;
		GLuint normal_tex;
		GLuint depth_tex;
		int fbo_width;
		int fbo_height;

		bool renderColor = false;

		Eigen::Matrix4f modelview_matrix;
		Eigen::Matrix4f projection_matrix;

		void InitResources();

	public:
		explicit Renderer(Window *window, bool renderColor);
		~Renderer();

		GLuint GetVertexTex()					{ return vertex_tex; }
		GLuint GetNormalTex()					{ return normal_tex; }
		Eigen::Matrix4f GetModelviewMatrix()	{ return modelview_matrix; }
		Eigen::Matrix4f GetProjectionMatrix()	{ return projection_matrix; }

		void Render(GLModel *model, Frame *frame, CameraTransform *camera_transform);
};

#endif //_RENDERER_H