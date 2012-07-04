

#include <blort/TomGine/tgFont.h>
#include <blort/TomGine/headers.h>
#include <stdexcept>
#include <stdio.h>

using namespace TomGine;

tgFont::tgFont(){
#ifdef USE_FTGL_FONT
	m_font = new FTPixmapFont(TTF_FONT);
	if(m_font->Error()){
		char err[64];
		sprintf(err, "[tgFont::tgFont()] Errof cannot create font '%s'", TTF_FONT);
		throw std::runtime_error(err);
	}
#else
	printf("[tgFont::tgFont] Warning: ftgl fonts disabled. USE_FTGL_FONT not defined\n");
#endif
}

tgFont::~tgFont(){
#ifdef USE_FTGL_FONT
	if(m_font)
		delete(m_font);
#endif
}

void tgFont::Print(const char* text, int size, int pos_x, int pos_y) const{
#ifdef USE_FTGL_FONT
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	m_font->FaceSize(size);
	//m_font->CharMap(ft_encoding_unicode);
	m_font->Render(text, -1, FTPoint(pos_x,pos_y));
	glDisable(GL_BLEND);
#endif
}

void tgFont::Print(const char* text, int size, int pos_x, int pos_y, float x, float y, float z, float a) const{
#ifdef USE_FTGL_FONT
	glColor4f(x,y,z,a);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	m_font->FaceSize(size);
	//m_font->CharMap(ft_encoding_unicode);
	m_font->Render(text, -1, FTPoint(pos_x,pos_y));
	glDisable(GL_BLEND);
#endif
}
