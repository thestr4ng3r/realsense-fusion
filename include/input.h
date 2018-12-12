
#ifndef _INPUT_H
#define _INPUT_H

class Frame;

class Input
{
	public:
		virtual ~Input() {}

		virtual bool WaitForFrame(Frame *frame) =0;
};

#endif //INPUT_H
