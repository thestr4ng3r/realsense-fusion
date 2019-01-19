
#ifndef _INPUT_H
#define _INPUT_H

class Frame;

class Input
{
	public:
		virtual ~Input() {}

		virtual bool WaitForFrame(Frame *frame) =0;
		virtual float GetPpx() = 0;
		virtual float GetPpy() = 0;
		virtual float GetFx() = 0;
		virtual float GetFy() = 0;

		virtual float GetPpxColor() = 0;
		virtual float GetPpyColor() = 0;
		virtual float GetFxColor() = 0;
		virtual float GetFyColor() = 0;

		virtual void setFilterActive(bool set) = 0;
};

#endif //INPUT_H
