 /**
 * @file Texture.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Class for managing textures (load from file, bind, copy, ...).
 */
 
#ifndef __TEXTURE_H__
#define __TEXTURE_H__

#include <blort/Tracker/headers.h>
#include <opencv2/core/core.hpp>

namespace Tracking{

/**
* @brief Class Texture
*/
class Texture
{
private:
	GLuint m_texture_id;
	unsigned m_width;
	unsigned m_height;
	int m_res_id;
	
public:
	Texture();
	~Texture();
	
	bool load(unsigned char* image_data, unsigned width, unsigned height, GLenum format=GL_BGR);
	bool load(const char* filename);
	bool save(const char* filename);
	bool getImageData(unsigned char* image_data);
	
	void bind(int stage=0);
	void copyTexImage2D(unsigned width, unsigned height);		// copy frame buffer pixels to texture
	void copyTexImage2D(int x, int y, unsigned width, unsigned height);
	void copyFromTexture(Texture* tex);
	void copyFromTexture(Texture* tex, int x, int y, unsigned w, unsigned h);
	
	inline GLuint getTextureID(){ return m_texture_id; }
	inline unsigned getWidth(){ return m_width; }
	inline unsigned getHeight(){ return m_height; }
	inline int getResID(){ return m_res_id; }
	inline void setResID(int id){ m_res_id = id; }
        cv::Mat toCvMat();
	
	
};

} // namespace Tracking

#endif
