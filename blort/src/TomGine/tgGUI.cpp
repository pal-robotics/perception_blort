

#include <blort/TomGine/tgGUI.h>
#include <blort/TomGine/tgPlot2D.h>
#include <stdio.h>
#include <GL/gl.h>

using namespace blortGLWindow;
using namespace TomGine;

tgGUI::tgGUI(unsigned width, unsigned height)
{
	m_width = width;
	m_height = height;
	
	m_window = new GLWindow(width, height, "GUI");

	// Set camera
	glMatrixMode(GL_PROJECTION);
	glOrtho(0.0, width, 0.0, height, 0.01, 1.0);
	float ft[16] = {	1.0f, 0.0f, 0.0f, 0.0f,
						0.0f, 1.0f, 0.0f, 0.0f,
						0.0f, 0.0f, 1.0f, 0.0f,
						0.0f, 0.0f,-1.0f, 1.0};
	
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(ft);
}

tgGUI::~tgGUI()
{
	if(m_window)
		delete(m_window);
}

bool tgGUI::InputControl(Event &event)
{
	switch(event.type){
		
		case TMGL_Press:
// 			printf("event.key.keysym: %x\n", event.key.keysym);
			switch(event.input){
				case TMGL_Escape:
					return false;
					break;
				default:
					break;
			}
			break;
			
		case TMGL_Release:

			break;
			
		case TMGL_Motion:

			break;
                default:
                        break;
	} // switch(event.type)
return true;
}

bool tgGUI::Update()
{
	bool quit = true;
	Event event;
	while(m_window->GetEvent(event)){
		quit = InputControl(event);
		if(quit==false)
			return false;
	}
	
	//for(unsigned i=0; i<m_elements.size(); i++){
	//	m_elements[i]->draw();
	//}
	
	m_window->Update();
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	return true;
}

void tgGUI::Screenshot(const char *filename) const
{
	char* data = (char*)malloc(3*m_width*m_height);
	
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
	
	FILE *ppm;
	ppm = fopen(filename,"w");
	fprintf(ppm,"P6\n%u %u\n255\n",m_width,m_height);
	fwrite(&data, 1, 3*m_width*m_height, ppm);
	fclose(ppm);
}
