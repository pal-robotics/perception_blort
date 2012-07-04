/**
* @file tgPlot2D.h
* @author Thomas Mörwald
* @date September 2010
* @version 0.1
* @brief Online tgPlot2D of data.
* @namespace TomGUI
*/

#ifndef TOMGUI_TGPLOT2D
#define TOMGUI_TGPLOT2D

#include <blort/TomGine/tgMathlib.h>
#include <blort/TomGine/tgTimer.h>
#include <blort/TomGine/tgFont.h>

#include <vector>

namespace TomGine{

class tgPlot2D
{
protected:
	int x,y;
	unsigned w,h;
	
	std::vector<float> m_buffer_x;
	std::vector<float> m_buffer_y_1;
	std::vector<float> m_buffer_y_2;
	std::vector<float> m_buffer_y_3;
	std::vector<float> m_buffer_y_4;
	unsigned m_buffer_size;
	
	float x_max, x_min, y_min, y_max;
	float x_scale, y_scale;
	
	TomGine::tgFont font;
	TomGine::tgTimer m_timer;
	
	float findmax(const std::vector<float> &x) const;
	float findmin(const std::vector<float> &x) const;
	
	void updateBuffer(float y1, float y2=0.0f, float y3=0.0f, float y4=0.0f);
	

public:

	tgPlot2D(int x, int y, unsigned w, unsigned h);
	~tgPlot2D();
	
	void axis(float x_min, float x_max, float y_min, float y_max);
	
	void setBufferSize(unsigned size){ m_buffer_size = size; }
	
	
	virtual void draw();
	
	void plot(const std::vector<float> &x, const std::vector<float> &y) const;
	
	void push(float y1);
	void push(float y1, float y2);
	void push(float y1, float y2, float y3);
	void push(float y1, float y2, float y3, float y4);
	
};

}

#endif
