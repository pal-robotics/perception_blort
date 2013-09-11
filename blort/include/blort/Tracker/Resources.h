
#ifndef __RESOURCES_H__
#define __RESOURCES_H__

#include <blort/Tracker/headers.h>
#include <blort/Tracker/Singleton.h>
#include <blort/Tracker/ImageProcessor.h>
#include <blort/Tracker/TrackerModel.h>
#include <blort/Tracker/ModelLoader.h>
#include <blort/Tracker/Texture.h>
#include <blort/Tracker/Shader.h>

#ifndef FN_LEN
#define FN_LEN 256
#endif

#define g_Resources Tracking::Resources::GetInstance()

namespace Tracking{

typedef std::vector<Shader*> ShaderList;
typedef std::vector<char*> NameList;


class Resources : public Singleton <Resources>
{
friend class Singleton <Resources>;
private:
	Resources();
	
	// Singleton Resources (one instance in programm)
	CvCapture* 			m_capture;
	IplImage* 			m_image;
	ImageProcessor* 	m_ip;
	
	// Resources lists
	ShaderList		m_shaderList;
	
	// Name lists
	NameList		m_shaderNameList;
	
	char			m_shaderPath[FN_LEN];
	
	bool			m_showlog;
	
	int				SearchName(NameList* list, const char* filename);

public:
	~Resources();
	static Resources* GetInstance(){
		return Singleton <Resources>::GetInstance ();
	}
	
	// Initialisation
	IplImage*				InitCapture(const char* file);
	IplImage* 				InitCapture(float width=640.0, float height=480.0, int camID = CV_CAP_ANY);
	ImageProcessor*			InitImageProcessor(int width, int height);

    // Release-functions
	void ReleaseCapture();
	void ReleaseImageProcessor();
    
    // Set-function
	void 	SetShaderPath(const char* path){ sprintf(m_shaderPath, "%s", path); }
	void	ShowLog(bool b){ m_showlog = b; }

	// Get-functions
	IplImage* 			GetNewImage();
	IplImage* 			GetImage();
	ImageProcessor* 	GetImageProcessor();
	
	Shader*					GetShader(int id);
	
	// Add-functions
	int		AddShader(	const char* shadername,
						const char* vertex_file = NULL,
						const char* fragment_file = NULL,
						const char* header = NULL);
	
	// Release-functions
	void ReleaseShader();
	void ReleaseShader(int id);
	
	// Search-functions
	int	SearchShaderName(const char* filename);
};

} // namespace Tracking

#endif
