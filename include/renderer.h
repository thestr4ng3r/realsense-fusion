
#ifndef _RENDERER_H
#define _RENDERER_H

struct RendererInternal;
class Model;

class Renderer
{
	private:
		RendererInternal *internal;
		bool should_terminate;

		void InitResources();

	public:
		Renderer();
		~Renderer();

		void UpdateModel(Model *model);
		void Update();

		bool GetShouldTerminate()	{ return should_terminate; }
};

#endif //_RENDERER_H