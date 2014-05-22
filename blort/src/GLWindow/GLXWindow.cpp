#include <blort/GLWindow/GLWindow.h>
#include <stdio.h>
#include <stdexcept>
#include <vector>

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

namespace blortGLWindow{

struct GLWindowImpl
{
#ifdef WIN32
  WNDCLASS        wc;
  HWND            hWnd;
  HDC             hDC;
  HGLRC           hRC;
  MSG             msg;
#endif

#ifdef LINUX
  Display                 *dpy;
  Window                  root;
  XVisualInfo               *vi;
  Colormap                  cmap;
  XSetWindowAttributes      swa;
  Window                    glWin;
  Window                    btWin;
  Atom                      wmDelete;
  GLXContext                glc;
  XWindowAttributes         gwa;
#endif

  void init(unsigned int width, unsigned int height, const char* name, bool visible)
  {
#ifdef LINUX
	GLint att[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None };
	dpy = XOpenDisplay(NULL);

	if(dpy == NULL){
		throw std::runtime_error("[GLWindow::init] Error cannot connect to X server");
	}
				  
	root = DefaultRootWindow(dpy);
	vi = glXChooseVisual(dpy, 0, att);

	if(vi == NULL)
		throw std::runtime_error("[GLWindow::init] Error no appropriate visual found");

	cmap = XCreateColormap(dpy, root, vi->visual, AllocNone);

	swa.colormap = cmap;
	swa.event_mask = ExposureMask | KeyPressMask | KeyReleaseMask | ButtonPressMask | ButtonReleaseMask | PointerMotionMask;

	glWin = XCreateWindow(dpy, root, 0, 0, width, height, 0, vi->depth, InputOutput, vi->visual, CWColormap | CWEventMask, &swa);
	wmDelete = XInternAtom(dpy, "WM_DELETE_WINDOW", true);
	XSetWMProtocols(dpy, glWin, &wmDelete, 1);

    if ( visible )
    {
      XMapWindow(dpy, glWin); //makes it visible
    }

	XStoreName(dpy, glWin, name);

	glc = glXCreateContext(dpy, vi, NULL, GL_TRUE);
	glXMakeCurrent(dpy, glWin, glc);
	
    printf("OpenGL Version: %s\n", glGetString(GL_VERSION));

	glXSwapBuffers(dpy, glWin);
    if(!visible)
    {
      XUnmapWindow(dpy, glWin);
    }
#endif // LINUX implementation
  }

  void quit()
  {
#ifdef LINUX
	glXMakeCurrent(dpy, None, NULL);
	glXDestroyContext(dpy, glc);
	XDestroyWindow(dpy, glWin);
	XCloseDisplay(dpy);
#endif // LINUX implementation
  }

  void Activate()
  {
#ifdef LINUX
	glXMakeCurrent(dpy, glWin, glc);
#endif // LINUX implementation
  }

  void Update()
  {
#ifdef LINUX
	glXSwapBuffers(dpy, glWin);
#endif // LINUX implementation
  }

  bool GetEvent(Event &event){
#ifdef LINUX
	if(XPending(dpy)){
		XEvent xev;
		
		// Keyboard Key Press (Keysym code available in /usr/include/X11/keysymdef.h)
		if( XCheckWindowEvent(dpy, glWin, KeyPressMask, &xev) )
		{
			event.type = TMGL_Press;
			KeySym ks = XKeycodeToKeysym(dpy, xev.xkey.keycode, 0);
			MapKey(ks, event.input);
		}
		// Keyboard Key Release (Keysym code available in /usr/include/X11/keysymdef.h)
		else if(XCheckWindowEvent(dpy, glWin, KeyReleaseMask, &xev))
		{
			event.type = TMGL_Release;
			KeySym ks = XKeycodeToKeysym(dpy, xev.xkey.keycode, 0);
			MapKey(ks, event.input);
		}
		// Mouse Button Press
		else if(XCheckWindowEvent(dpy, glWin, ButtonPressMask, &xev))
		{
			event.type = TMGL_Press;
			MapMouse(xev.xbutton.button, event.input);
            event.motion.x = xev.xbutton.x;
            event.motion.y = xev.xbutton.y;
		}
		// Mouse Button Release
		else if(XCheckWindowEvent(dpy, glWin, ButtonReleaseMask, &xev))
		{
			event.type = TMGL_Release;
			MapMouse(xev.xbutton.button, event.input);
            event.motion.x = xev.xbutton.x;
            event.motion.y = xev.xbutton.y;
		}
		// Mouse Motion
		else if(XCheckWindowEvent(dpy, glWin, PointerMotionMask, &xev))
		{
			event.type = TMGL_Motion;
			event.motion.x = xev.xmotion.x;
			event.motion.y = xev.xmotion.y;
		}
		// Window Exposure
		else if(XCheckWindowEvent(dpy, glWin, ExposureMask, &xev))
		{
			XWindowAttributes gwa;
			event.type = TMGL_Expose;
			XGetWindowAttributes(dpy, glWin, &gwa);
			event.expose.width = gwa.width;
			event.expose.height = gwa.height;
		}
		
		// Other Events
		else
		{
			return false;
		}

		return true;
	}
#endif // LINUX implementation
	return false;
  }

#ifdef LINUX
  void MapMouse(unsigned int xbutton, Input &input)
  {
  	switch(xbutton)
  	{
  		case Button1:
  			input = TMGL_Button1;
  			break;
  		case Button2:
  			input = TMGL_Button2;
  			break;
  		case Button3:
  			input = TMGL_Button3;
  			break;
  		case Button4:
  			input = TMGL_Button4;
  			break;
  		case Button5:
  			input = TMGL_Button5;
  			break;
  	}
  }

  void MapKey(KeySym ks, Input &input)
  {
  	switch(ks)
  	{
  		case XK_0:
  			input = TMGL_0;
  			break;
  		case XK_1:
  			input = TMGL_1;
  			break;
  		case XK_2:
  			input = TMGL_2;
  			break;
  		case XK_3:
  			input = TMGL_3;
  			break;
  		case XK_4:
  			input = TMGL_4;
  			break;
  		case XK_5:
  			input = TMGL_5;
  			break;
  		case XK_6:
  			input = TMGL_6;
  			break;
  		case XK_7:
  			input = TMGL_7;
  			break;
  		case XK_8:
  			input = TMGL_8;
  			break;
  		case XK_9:
  			input = TMGL_9;
  			break;
  			
  		case XK_a:
  			input = TMGL_a;
  			break;
  		case XK_b:
  			input = TMGL_b;
  			break;
  		case XK_c:
  			input = TMGL_c;
  			break;
  		case XK_d:
  			input = TMGL_d;
  			break;
  		case XK_e:
  			input = TMGL_e;
  			break;
  		case XK_f:
  			input = TMGL_f;
  			break;
  		case XK_g:
  			input = TMGL_g;
  			break;
  		case XK_h:
  			input = TMGL_h;
  			break;
  		case XK_i:
  			input = TMGL_i;
  			break;
  		case XK_j:
  			input = TMGL_j;
  			break;
  		case XK_k:
  			input = TMGL_k;
  			break;
  		case XK_l:
  			input = TMGL_l;
  			break;
  		case XK_m:
  			input = TMGL_m;
  			break;
  		case XK_n:
  			input = TMGL_n;
  			break;
  		case XK_o:
  			input = TMGL_o;
  			break;
  		case XK_p:
  			input = TMGL_p;
  			break;
  		case XK_q:
  			input = TMGL_q;
  			break;
  		case XK_r:
  			input = TMGL_r;
  			break;
  		case XK_s:
  			input = TMGL_s;
  			break;
  		case XK_t:
  			input = TMGL_t;
  			break;
  		case XK_u:
  			input = TMGL_u;
  			break;
  		case XK_v:
  			input = TMGL_v;
  			break;
  		case XK_w:
  			input = TMGL_w;
  			break;
  		case XK_x:
  			input = TMGL_x;
  			break;
  		case XK_y:
  			input = TMGL_y;
  			break;
  		case XK_z:
  			input = TMGL_z;
  			break;
  		
  		case XK_space:
  			input = TMGL_Space;
  			break;
  		case XK_BackSpace:
  			input = TMGL_BackSpace;
  			break;
  		case XK_Tab:
  			input = TMGL_Tab;
  			break;
  		case XK_Return:
  			input = TMGL_Return;
  			break;
  		case XK_Pause:
  			input = TMGL_Pause;
  			break;
  		case XK_Escape:
  			input = TMGL_Escape;
  			break;
  		case XK_Delete:
  			input = TMGL_Delete;
  			break;
  		case XK_Left:
  			input = TMGL_Left;
  			break;
  		case XK_Up:
  			input = TMGL_Up;
  			break;
  		case XK_Right:
  			input = TMGL_Right;
  			break;
  		case XK_Down:
  			input = TMGL_Down;
  			break;
  		case XK_Page_Up:
  			input = TMGL_Page_Up;
  			break;
  		case XK_Page_Down:
  			input = TMGL_Page_Down;
  			break;
  		case XK_End:
  			input = TMGL_End;
  			break;
  		case XK_Begin:
  			input = TMGL_Begin;
  			break;
  		case XK_KP_Enter:
  			input = TMGL_Return;
  			break;
  		case XK_KP_Multiply:
  			input = TMGL_KP_Multiply;
  			break;
  		case XK_KP_Add:
  			input = TMGL_KP_Add;
  			break;
  		case XK_KP_Subtract:
  			input = TMGL_KP_Subtract;
  			break;
  		case XK_KP_Divide:
  			input = TMGL_KP_Divide;
  			break;
  		case XK_KP_0:
  			input = TMGL_KP_0;
  			break;
  		case XK_KP_1:
  			input = TMGL_KP_1;
  			break;
  		case XK_KP_2:
  			input = TMGL_KP_2;
  			break;
  		case XK_KP_3:
  			input = TMGL_KP_3;
  			break;
  		case XK_KP_4:
  			input = TMGL_KP_4;
  			break;
  		case XK_KP_5:
  			input = TMGL_KP_5;
  			break;
  		case XK_KP_6:
  			input = TMGL_KP_6;
  			break;
  		case XK_KP_7:
  			input = TMGL_KP_7;
  			break;
  		case XK_KP_8:
  			input = TMGL_KP_8;
  			break;
  		case XK_KP_9:
  			input = TMGL_KP_9;
  			break;
  		case XK_F1:
  			input = TMGL_F1;
  			break;
  		case XK_F2:
  			input = TMGL_F2;
  			break;
  		case XK_F3:
  			input = TMGL_F3;
  			break;
  		case XK_F4:
  			input = TMGL_F4;
  			break;
  		case XK_F5:
  			input = TMGL_F5;
  			break;
  		case XK_F6:
  			input = TMGL_F6;
  			break;
  		case XK_F7:
  			input = TMGL_F7;
  			break;
  		case XK_F8:
  			input = TMGL_F8;
  			break;
  		case XK_F9:
  			input = TMGL_F9;
  			break;
  		case XK_F10:
  			input = TMGL_F10;
  			break;
  		case XK_F11:
  			input = TMGL_F11;
  			break;
  		case XK_F12:
  			input = TMGL_F12;
  			break;
  		case XK_Shift_L:
  			input = TMGL_Shift_L;
  			break;
  		case XK_Shift_R:
  			input = TMGL_Shift_R;
  			break;
  		case XK_Control_L:
  			input = TMGL_Control_L;
  			break;
  		case XK_Control_R:
  			input = TMGL_Control_R;
  			break;
  	}
  }
#endif // LINUX implementation for Mouse/Keyboard mapping
};

void GLWindow::init(unsigned int width, unsigned int height, const char* name, bool visible){
    impl->init(width, height, name, visible);
}

void GLWindow::quit(){
    impl->quit();
}

void GLWindow::Activate(){
    impl->Activate();
}

void GLWindow::Update(){
    impl->Update();
}

GLWindow::GLWindow()
: impl(new GLWindowImpl())
{
	init(320,240,"OpenGL Window");
}

GLWindow::GLWindow(unsigned int width, unsigned int height)
: impl(new GLWindowImpl())
{
	init(width, height, "OpenGL Window");
}

GLWindow::GLWindow(unsigned int width, unsigned int height, const char* name, bool visible)
: impl(new GLWindowImpl())
{
  init(width, height, name, visible);
}
GLWindow::~GLWindow(){
	quit();
    delete impl;
}

bool GLWindow::GetEvent(Event &event){
    return impl->GetEvent(event);
}

} /* namespace */
