#include <ros/console.h>
#include <blort/Tracker/Resources.h>

using namespace Tracking;

// *** PRIVATE ***

int Resources::SearchName(NameList* list, const char* filename){
	// parse through name list
	int i = 0;
	NameList::iterator it_name = list->begin();
	while(it_name != list->end()){
		if(!strcmp((*it_name),filename))
			return i;
		++it_name;
		++i;
	}
	
	// not found
	return -1;
}

// *** PUBLIC ***

Resources::Resources(){
	m_capture = 0;
	m_image = 0;
	m_ip = 0;
	m_showlog = false;
}

Resources::~Resources(){
	ReleaseCapture();
	//ReleaseScreen();
	ReleaseImageProcessor();
	
	ReleaseShader();
	
        if(m_showlog) ROS_DEBUG("Resources released");
}


// *** Initialisation ***
IplImage* Resources::InitCapture(const char* file){
	m_capture = cvCaptureFromFile(file);
	if(!m_capture) {
		char errmsg[128];
		sprintf(errmsg, "[Resources::InitCapture] Error could not read '%s'\n", file );
		throw std::runtime_error(errmsg);
	}
	
	double w = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_WIDTH );
	double h = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_HEIGHT );
	
        if(m_showlog) ROS_DEBUG("Camera settings: %.1f x %.1f", w, h);
	
	m_image = cvQueryFrame(m_capture);
	cvConvertImage(m_image, m_image, CV_CVTIMG_FLIP | CV_CVTIMG_SWAP_RB);
	//cvFlip(m_image, m_image, 1);
	return m_image;
}

IplImage* Resources::InitCapture(float width, float height, int camID){
	m_capture = cvCreateCameraCapture(camID);
	if(!m_capture) {
		char errmsg[128];
		sprintf(errmsg, "[Resources::InitCapture] Error could not initialise camera\n" );
		throw std::runtime_error(errmsg);
	}
	
	cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_WIDTH, width );
	cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_HEIGHT, height );
	
	double w = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_WIDTH );
	double h = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_HEIGHT );
	
        if(m_showlog) ROS_DEBUG("Camera settings: %.1f x %.1f", w, h);
	
	m_image = cvQueryFrame(m_capture);
	cvConvertImage(m_image, m_image, CV_CVTIMG_FLIP | CV_CVTIMG_SWAP_RB);
	//cvFlip(m_image, m_image, 1);
	return m_image;
}

ImageProcessor* Resources::InitImageProcessor(int width, int height){
	if(width==0||height==0){
                ROS_DEBUG("[Resources::GetImageProcessor] Error ImageProcessor needs width and height for initialisation");
		return 0;
	}
	
	if(!m_ip)
		m_ip = new(ImageProcessor);
		
	if( !(m_ip->init(width, height)) ){
                ROS_DEBUG("[Resources::GetImageProcessor] Error could not initialise ImageProcessor");
		delete(m_ip);
		m_ip = 0;
		return 0;
	}

	return m_ip;
}

// *** Release-functions ***
void Resources::ReleaseCapture(){
	if(m_capture)
		cvReleaseCapture(&m_capture);
	m_capture = 0;
}

void Resources::ReleaseImageProcessor(){
	if(m_ip)
		delete(m_ip);
	m_ip = 0;
}


// *** Get-functions ***
IplImage* Resources::GetNewImage(){
	
	if(!m_capture){
                ROS_DEBUG("[Resources::GetNewImage] Error camera not initialised" );
		return 0;
	}
        IplImage* img = 0;
	
	try{
		img = cvQueryFrame(m_capture);
	}
	catch(char const* e){
                ROS_DEBUG("[Resources::GetNewImage()] Warning: %s", e);
	}
	
	if(img != NULL){
		m_image = img;
// 		cvConvertImage(m_image, m_image, CV_CVTIMG_SWAP_RB);
	}
	
	//cvFlip(m_image, m_image, 1);
	return m_image;
}

IplImage* Resources::GetImage(){
	if(!m_image){
		return GetNewImage();
	}
	return m_image;
}

ImageProcessor* Resources::GetImageProcessor(){
	if(!m_ip){
                ROS_DEBUG("[Resources::GetImageProcessor] Error ImageProcessor not initialised" );
		return 0;
    }
	return m_ip;	
}

Shader* Resources::GetShader(int id){
	return m_shaderList[id];
}

// *** Add
int	Resources::AddShader(	const char* shadername,
							const char* vertex_file,
							const char* fragment_file,
							const char* header)
{
	int shaderID=-1;
	/*
	// check if texture allready loaded before by comparing filename
	int shaderID = SearchShaderName(shadername);
	if(shaderID != -1)
		return shaderID;	// return existing texture ID
	*/
	
	// texture doesn't exist and needs to be loaded
	char vertex_fullname[FN_LEN];
	char fragment_fullname[FN_LEN];
	char header_fullname[FN_LEN];
	sprintf(vertex_fullname, "%s%s", m_shaderPath, vertex_file);
	sprintf(fragment_fullname, "%s%s", m_shaderPath, fragment_file);
	sprintf(header_fullname, "%s%s", m_shaderPath, header);
		
	if(vertex_file)
		vertex_file = &vertex_fullname[0];
	if(fragment_file)
		fragment_file = &fragment_fullname[0];
	if(header)
		header = &header_fullname[0];

	Shader* shader = new Shader(vertex_file, fragment_file, header);

	if(!shader->getStatus()){
                ROS_DEBUG("[Resources::AddShader] Error failed to load shader %s", shadername);
                ROS_DEBUG("[Resources::AddShader]   Vertex shader: '%s'", vertex_fullname);
                ROS_DEBUG("[Resources::AddShader]   Fragment shader: '%s'", fragment_fullname);
                ROS_DEBUG("[Resources::AddShader]   Header shader: '%s'", header_fullname);
		delete(shader);
		return -1;
	}
	
	char* name = new char[FN_LEN];
	strcpy(name, shadername);
	
	// put model into texture list
	m_shaderNameList.push_back(name);
	m_shaderList.push_back(shader);
	
	shaderID = m_shaderList.size()-1;
	
        if(m_showlog) ROS_DEBUG("Shader %i loaded: %s", shaderID, name);
	
	return shaderID;
}

// *** Release
void Resources::ReleaseShader(){
	// release Shader
        for(unsigned int i = 0; i<m_shaderList.size(); ++i)
            delete(m_shaderList[i]);
        m_shaderList.clear();
	// release Shadernames
        for(unsigned int i = 0; i<m_shaderNameList.size(); ++i)
            delete(m_shaderNameList[i]);
        m_shaderNameList.clear();
}

void Resources::ReleaseShader(int id){
	delete(m_shaderList[id]);
	m_shaderList[id] = 0;
        if(m_showlog) ROS_DEBUG("Shader %i released", id);
}


// *** Search-functions ***
int	Resources::SearchShaderName(const char* filename){
	return SearchName(&m_shaderNameList, filename);
}
