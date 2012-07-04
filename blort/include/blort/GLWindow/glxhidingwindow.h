#ifndef GLXHIDINGWINDOW_H
#define GLXHIDINGWINDOW_H

#include <blort/GLWindow/GLWindow.h>

namespace pal_blort
{
    class GLXHidingWindow : public ::blortGLWindow::GLWindow
    {
    public:
        GLXHidingWindow();
        GLXHidingWindow(unsigned int width, unsigned int height);
        GLXHidingWindow(unsigned int width, unsigned int height, const char* name);
  };
}
#endif // GLXHIDINGWINDOW_H
