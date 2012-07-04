 /**
 * @file GLEvent.h
 * @author Thomas Moerwald (Vienna University of Technology)
 * @date June 2010
 * @version 0.1
 * @brief Structure defining keyboard and mouse input events.
 */

#ifndef _GL_WINDOW_EVENT_H_
#define _GL_WINDOW_EVENT_H_

#include <blort/GLWindow/GLInput.h>

namespace blortGLWindow{

/** @brief Input event class */
class Event{
public:
	struct Motion{
		int x,y;
	};
	struct Exposure{
		int width,height;
	};
	
	Type		type;		///> Type of input event (defined in GLInput.h)
	Input		input;	///> Key- or mouse constant (defined in GLInput.h)
	Motion		motion; ///> Mouse position in pixels if event is of type TMGL_Motion
	Exposure	expose; ///> Exposure parameter if event is of type TMGL_Expose
	
};

} /* namespace */

#endif /* _GL_WINDOW_EVENT_H_ */
