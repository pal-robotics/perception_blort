 /**
 * @file GLWindow.h
 * @author Thomas Moerwald (Vienna University of Technology)
 * @date June 2010
 * @version 0.1
 * @brief Device Context for handling OpenGL windows in MS GLWindows.
 */
 
#ifndef _GL_WINDOW_
#define _GL_WINDOW_


#include <blort/GLWindow/GLEvent.h>

/** @brief BLORT namespace for GLWindow */
namespace blortGLWindow{

#ifdef WIN32
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
#endif

struct GLWindowImpl;

/** @brief Class GLWindow */
class GLWindow{

public:
	/** @brief Construction of an OpenGL Window (Rendering Context)
	*   @param width Window and OpenGL viewport width in pixel
	*   @param height Window and OpenGL viewport height in pixel
  *   @param name Caption of the window in the titel bar
  *   @param visible if false the window will not be mapped to the display */
  GLWindow();
  GLWindow(unsigned int width, unsigned int height);
  GLWindow(unsigned int width, unsigned int height, const char* name, bool visible = true);
  ~GLWindow();

  /** @brief Activate window for usage (set focus) */
  void Activate();

  /** @brief Update OpenGL GLWindow (Swap Buffers) */
  void Update();

  /** @brief Query input event 
  *   @return true if there are events in the event que 
  *		@param Event defined in TMGLEvent.h */
  bool GetEvent(Event &event);

#ifdef WIN32
  HWND gethWnd() const {return hWnd;}
#endif

protected:
  GLWindowImpl * impl;

  void init(unsigned int width, unsigned int height, const char* name, bool visible = true);
  void quit();
  
};

} /* namespace */

#endif /* _GL_WINDOW_ */
