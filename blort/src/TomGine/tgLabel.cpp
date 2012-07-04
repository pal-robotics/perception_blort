

#include <blort/TomGine/tgLabel.h>
#include <stdexcept>

using namespace TomGine;
using namespace std;

tgLabel::tgLabel(){
// 	m_maxStrLen=0;
// 	m_texture = 0;
// 	m_font = 0;
// 	m_txtSize = 0;
}

tgLabel::tgLabel(const char* ttf_filename){
// 	m_fontfilename = string(ttf_filename);
// 	m_maxStrLen=0;
// 	m_texture = 0;
// 	m_font = 0;
// 	m_txtSize = 0;
}

tgLabel::~tgLabel(){
// 	if(m_font){
// 		delete(m_font);
// 	}
// 	
// 	if(m_texture){
// 		delete(m_texture);
// 	}
}

void tgLabel::AddText(const char* text, unsigned size){
// 	m_txtHeight = size;
// 	string str(text);
// 	m_text.push_back(str);
// 	if(m_maxStrLen<str.size())
// 		m_maxStrLen = str.size();
// 		
// 	CreateLabel();
}

void tgLabel::CreateLabel(){
// 	if(m_fontfilename.empty()){
// 		printf("[tgLabel::CreateLabel()] Warning no font specified for label\n");
// 		return;
// 	}	
// 
// 	unsigned i,s;
// 	unsigned m_charWidth = (unsigned)floor(m_txtHeight * 0.75f); //15;
// 	unsigned m_txtX = (unsigned)floor(m_txtHeight * 0.2f); //4;
// 	unsigned m_txtY = (unsigned)floor(m_txtHeight * 0.4f); //8;
// 	unsigned m_txtStepY = m_txtHeight + (unsigned)floor(m_txtHeight * 0.2f); //24;
// 
// 	if(!m_texture)
// 		m_texture = new tgTexture();
// 
// 	if(!m_font)
// 		m_font = new tgFont(m_fontfilename.c_str());
// 	
// 	glDisable(GL_DEPTH_TEST);
// 	glDepthMask(0);
// 	
// 	glClear(GL_COLOR_BUFFER_BIT);
// 	
// 	s = m_text.size();
// 	for(i=0; i<s; i++){
// 		m_font->Print(m_text[s-i-1].c_str(), m_txtHeight, m_txtX, m_txtY+m_txtStepY*i);
// 	}
// 	
// 	m_texture->CopyTexImage2D(m_charWidth*m_maxStrLen, m_txtY+m_txtStepY*i);
// 	
// 	m_width = m_charWidth*m_maxStrLen * 0.0005f;
// 	m_height = (m_txtY+m_txtStepY*i) * 0.0005f;
// 	
// 	glDepthMask(1);
// 	glEnable(GL_DEPTH_TEST);
// 	
// 	m_txtSize = m_text.size();
}

void tgLabel::Clear(){
// 	m_text.clear();
// 	m_maxStrLen = 0;
// 	m_txtSize = 0;
// 		
// 	if(m_texture){
// 		delete(m_texture);
// 		m_texture = 0;
// 	}
}

void tgLabel::Draw() const{

// 	if(m_text.empty())
// 		return;
// 
// 	if(m_txtSize < m_text.size())
// 		return;
// 
// 	float w=m_width;
// 	float h=m_height;
// 	vec3 vX;
// 	vec3 vY;
// 	vec3 vPos = m_pose.t;
// 	mat4 matrix;
// 
// 	glMatrixMode(GL_MODELVIEW);
// 	glPushMatrix();
// 	glGetFloatv( GL_MODELVIEW_MATRIX, matrix );
// 	vX = vec3(matrix[0], matrix[4], matrix[8]);
// 	vY = vec3(matrix[1], matrix[5], matrix[9]);
// 	vX = vX * w * 0.5;
// 	vY = vY * h * 0.5;
// 
// 
// 	glEnable(GL_BLEND);
// 	glBlendFunc(GL_SRC_COLOR, GL_ONE);
// // 	glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_ONE_MINUS_SRC_COLOR);
// 	glDisable(GL_DEPTH_TEST);
// 	glDisable(GL_LIGHTING);
// 	m_texture->Bind();
// 	
// 	glColor3f(1.0,1.0,1.0);
// 	glBegin(GL_QUADS);
// 		glTexCoord2f(0.0,0.0); glNormal3f(0.0, 0.0, 1.0); glVertex3f( vPos.x - vX.x - vY.x, vPos.y - vX.y - vY.y, vPos.z - vX.z - vY.z );
// 		glTexCoord2f(1.0,0.0); glNormal3f(0.0, 0.0, 1.0); glVertex3f( vPos.x + vX.x - vY.x, vPos.y + vX.y - vY.y, vPos.z + vX.z - vY.z );
// 		glTexCoord2f(1.0,1.0); glNormal3f(0.0, 0.0, 1.0); glVertex3f( vPos.x + vX.x + vY.x, vPos.y + vX.y + vY.y, vPos.z + vX.z + vY.z );
// 		glTexCoord2f(0.0,1.0); glNormal3f(0.0, 0.0, 1.0); glVertex3f( vPos.x - vX.x + vY.x, vPos.y - vX.y + vY.y, vPos.z - vX.z + vY.z );
// 	glEnd();
// 	
// 	glDisable(GL_TEXTURE_2D);
// 	glEnable(GL_LIGHTING);
// 	glEnable(GL_DEPTH_TEST);
// 	glDisable(GL_BLEND);
// 	
// 	glPopMatrix();

}

