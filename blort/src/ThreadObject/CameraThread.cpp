
#include <blort/ThreadObject/CameraThread.h>
#include <opencv2/legacy/compat.hpp>
#include <stdexcept>


CCameraThread::CCameraThread(int camID, int width, int height)
{
	m_mutex.Lock();
		m_new_image = false;
		m_camID = camID;
		m_width = width;
		m_height = height;
		
		m_capture = cvCreateCameraCapture(m_camID);
		
		if(!m_capture)
			throw std::runtime_error("[CCameraThread::CCameraThread] Can not create camera capture.\n");
		
		cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_WIDTH, m_width );
		cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_HEIGHT, m_height );
		m_image = cvQueryFrame(m_capture);
	m_mutex.Unlock();
}

CCameraThread::~CCameraThread()
{
	this->Stop();
}

BOOL CCameraThread::OnTask()
{
	m_image = cvQueryFrame(m_capture);
	m_new_image = true;
	return TRUE;
}

bool CCameraThread::GetImage(IplImage* image)
{
	if(m_new_image){
		cvCopyImage(m_image, image);
		m_new_image = false;
		return true;
	}
	return false;
}

