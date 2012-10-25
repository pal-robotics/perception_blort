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

#ifdef WIN32
#include <windows.h>
#include <gl/glew.h>
#include <gl/gl.h>
#endif

#ifdef LINUX
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/gl.h>
#include <GL/glx.h>
#endif

/** @brief BLORT namespace for GLWindow */
namespace blortGLWindow{

#ifdef WIN32
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
#endif

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
#ifdef WIN32
	WNDCLASS		wc;
	HWND 			hWnd;
	HDC 			hDC;
	HGLRC 			hRC;
	MSG 			msg;
#endif

#ifdef LINUX
  Display                 *dpy;
  Window                  root;
  XVisualInfo				*vi;
  Colormap					cmap;
  XSetWindowAttributes		swa;
  Window					glWin;
  Window					btWin;
  Atom						wmDelete;
  GLXContext				glc;
  XWindowAttributes     	gwa;
#endif

  void init(unsigned int width, unsigned int height, const char* name, bool visible = true);
  void quit();
  
};

} /* namespace */

#endif /* _GL_WINDOW_ */
