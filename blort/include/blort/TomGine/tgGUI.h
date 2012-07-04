/**
* @file TomtgGUI.h
* @author Thomas Mörwald
* @date September 2010
* @version 0.1
* @brief OpenGL tgGUI
* @namespace TomtgGUI
*/

#ifndef TOMGINE_TGGUI
#define TOMGINE_TGGUI


#include <blort/GLWindow/GLWindow.h>

#include <vector>

namespace TomGine{

class tgGUI
{
private:
	unsigned m_width, m_height;
	
	blortGLWindow::GLWindow* m_window;
	
	bool InputControl(blortGLWindow::Event &event);


public:
	tgGUI(unsigned width, unsigned height);
	~tgGUI();
	
	bool Update();
	
	void Activate(){ m_window->Activate(); }
	
	
	void GetPlot2D(int x, int y, unsigned w, unsigned h);

	void Screenshot(const char* filename) const;

};

}


#endif
