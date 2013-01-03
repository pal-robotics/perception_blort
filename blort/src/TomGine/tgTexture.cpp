
#include <blort/TomGine/tgTexture.h>
#include <blort/TomGine/tgError.h>
#include <opencv2/highgui/highgui.hpp>
#include <stdexcept>

using namespace TomGine;

tgTexture::tgTexture(){
	glGenTextures(1, &m_texture_id);
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
	tgCheckError("[tgTexture::tgTexture()]");
}

tgTexture::~tgTexture(){
	if(glIsTexture(m_texture_id))
		glDeleteTextures(1, &m_texture_id);
}

bool tgTexture::Load(void* image_data, int width, int height, GLenum format, GLenum internal){
	m_width = width;
	m_height = height;
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, (int)m_width, (int)m_height, 0, format, internal, image_data);
	return true;
}

bool tgTexture::Load(const char* filename){
	IplImage* img = cvLoadImage(filename, CV_LOAD_IMAGE_COLOR);
	if(img==NULL){
		std::string errmsg = std::string("[tgTexture::Load] Error loading file '") + filename + "'.";
		throw std::runtime_error(errmsg.c_str());
	}
	return Load((unsigned char*)img->imageData, img->width, img->height, GL_BGR);
}

bool tgTexture::Save(const char* filename, bool overwrite){
	Bind();
	IplImage* img = cvCreateImage ( cvSize ( m_width, m_height ), IPL_DEPTH_8U, 3 );
	glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR, GL_UNSIGNED_BYTE, img->imageData);
	cvSaveImage(filename, img);
	cvReleaseImage(&img);
	return true;
}

bool tgTexture::GetImageData(unsigned char* image_data){
	Bind();
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, image_data);
	return true;
}

void tgTexture::Bind(int stage) const{
	glActiveTexture(GL_TEXTURE0 + stage);
	glBindTexture(GL_TEXTURE_2D, m_texture_id);
	glEnable(GL_TEXTURE_2D);
}

void tgTexture::CopyTexImage2D(int width, int height){
	m_width = width;
	m_height = height;
	Bind();
	glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 0, 0, m_width, m_height, 0);	
}

void tgTexture::CopyTexImage2D(int x, int y, int width, int height){
	m_width = width;
	m_height = height;
	Bind();
	glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, x, y, m_width, m_height, 0);	
}


