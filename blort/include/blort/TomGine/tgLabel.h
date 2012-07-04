 /**
 * @file tgLabel.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Quad patch with texture created by copying text from frame buffer.
 * @namespace TomGine
 */
 
#ifndef _TG_LABEL_H_
#define _TG_LABEL_H_
 
#include <vector>
#include <string>

#include <blort/TomGine/tgFont.h>
#include <blort/TomGine/tgTexture.h>
#include <blort/TomGine/tgPose.h>

namespace TomGine{

class tgLabel
{
private:
	float m_width;
	float m_height;
	unsigned m_txtSize;
	unsigned m_txtHeight;
	unsigned m_maxStrLen;
	
	tgPose m_pose;
	tgFont* m_font;
	tgTexture* m_texture;
	std::vector<std::string> m_text;
	std::string m_fontfilename;

public:
	
	
	/** @brief Constructor of tgLabel
	*	@param w width of label in pixels
	* @param h height of label in pixels
	* @param ttf_filename file name of true type font (ttf) to use for text */
	tgLabel();
	tgLabel(const char* ttf_filename);
	~tgLabel();
	
	void SetFont(const char* ttf_filename){ m_fontfilename = std::string(ttf_filename); }
	void SetPose(tgPose pose){ m_pose = pose; }
	
	/** @brief adds text to the label, should not be called at framerate, since it creates a texture on the GPU 
	*/
	void AddText(const char* text, unsigned size=20);
	void CreateLabel();
	
	/** @brief clears the text of the label and destroys texture for it on the GPU
	*/
	void Clear();
	
	/** @brief draws the label at frame rate, should be called after everything else is rendered, since depth-testing is disabled
	*	
	*/
	void Draw() const;
	const char* c_str(){ if(!m_text.empty()) return m_text[0].c_str(); }
	
};

}

#endif
