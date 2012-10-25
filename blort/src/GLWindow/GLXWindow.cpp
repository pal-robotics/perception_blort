
#ifdef LINUX

#include <blort/GLWindow/GLWindow.h>
#include <stdio.h>
#include <stdexcept>
#include <vector>

namespace blortGLWindow{

void GLWindow::init(unsigned int width, unsigned int height, const char* name, bool visible){
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
    XMapWindow(dpy, glWin); //makes it visible

	XStoreName(dpy, glWin, name);

	glc = glXCreateContext(dpy, vi, NULL, GL_TRUE);
	glXMakeCurrent(dpy, glWin, glc);
	
        printf("OpenGL Version: %s\n", glGetString(GL_VERSION));

	glXSwapBuffers(dpy, glWin);
}

void GLWindow::quit(){
	glXMakeCurrent(dpy, None, NULL);
	glXDestroyContext(dpy, glc);
	XDestroyWindow(dpy, glWin);
	XCloseDisplay(dpy);
}

GLWindow::GLWindow(){
	init(320,240,"OpenGL Window");
}
GLWindow::GLWindow(unsigned int width, unsigned int height){
	init(width, height, "OpenGL Window");
}
GLWindow::GLWindow(unsigned int width, unsigned int height, const char* name, bool visible){
  init(width, height, name, visible);
}
GLWindow::~GLWindow(){
	quit();
}

void GLWindow::Activate(){
	glXMakeCurrent(dpy, glWin, glc);
}

void GLWindow::Update(){
	glXSwapBuffers(dpy, glWin);
}

} /* namespace */

#endif /* LINUX */

