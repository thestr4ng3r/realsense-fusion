
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
		GLint enable_color_uniform = -1;
		GLint enable_lighting_uniform = -1;
		GLint drift_correction_uniform = -1;

		GLuint box_program = 0;
		GLint box_mvp_matrix_uniform = -1;

		GLuint fbo;
		GLuint color_tex;
		GLuint vertex_tex;
		GLuint normal_tex;
		GLuint depth_tex;
		int fbo_width;
		int fbo_height;

		bool enable_color = false;
		bool enable_lighting = true;

		Eigen::Matrix4f modelview_matrix;
		Eigen::Matrix4f projection_matrix;

		Eigen::Vector3f drift_correction;

		void InitResources();

	public:
		explicit Renderer(Window *window);
		~Renderer();

		GLuint GetVertexTex()						{ return vertex_tex; }
		GLuint GetNormalTex()						{ return normal_tex; }
		Eigen::Matrix4f GetModelviewMatrix()		{ return modelview_matrix; }
		Eigen::Matrix4f GetProjectionMatrix()		{ return projection_matrix; }

		void Render(GLModel *model, Frame *frame, CameraTransform *camera_transform);

		bool GetEnableColor()						{ return enable_color; }
		bool GetEnableLighting()					{ return enable_lighting; }

		void SetEnableColor(bool v)					{ enable_color = v; }
		void SetEnableLighting(bool v)				{ enable_lighting = v; }

		Eigen::Vector3f GetDriftCorrection()		{ return drift_correction; }
		void SetDriftCorrection(Eigen::Vector3f v)	{ drift_correction = v; }
};

#endif //_RENDERER_H