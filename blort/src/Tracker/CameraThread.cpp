
#include <blort/Tracker/CameraThread.h>
#include <stdexcept>

using namespace Tracking;

CameraThread::CameraThread(int camID, int width, int height)
{
	m_mutex.Lock();
		m_new_image = false;
		m_camID = camID;
		m_width = width;
		m_height = height;
		
		m_capture = cvCreateCameraCapture(m_camID);
		
		if(!m_capture)
			throw std::runtime_error("[CameraThread::CameraThread] Can not create camera capture.\n");
		
		cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_WIDTH, m_width );
		cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_HEIGHT, m_height );
		m_image = cvQueryFrame(m_capture);
		
		if(m_image->width != width || m_image->height != height){
			char errmsg[128];
			sprintf(errmsg, "[CameraThread::CameraThread] Created capture with wrong resolution %dx%d instead of %dx%d (OpenCV)", m_image->width, m_image->height, width, height);
			throw std::runtime_error(errmsg);			
		}
		
	m_mutex.Unlock();
}

CameraThread::~CameraThread()
{
	this->Stop();
}

BOOL CameraThread::OnTask()
{
	m_image = cvQueryFrame(m_capture);
	m_new_image = true;
	return TRUE;
}

void CameraThread::GetImage(IplImage* image)
{
	if(m_new_image){
// 		printf("%d %d, %d %d\n", m_image->width, m_image->height, image->width, image->height);
		cvCopyImage(m_image, image);
		m_new_image = false;
	}
}

