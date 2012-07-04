
#include <blort/Tracker/TrackerThread.h>
#include <blort/Tracker/TextureTracker.h>
#include <blort/GLWindow/GLWindow.h>
#include <blort/Tracker/CameraThread.h>
#include <stdexcept>

using namespace Tracking;
using namespace std;

TrackerThread::TrackerThread()
{
	m_mutex.Lock();
		m_quit = false;
		cmd = IDLE;
	m_mutex.Unlock();
}

TrackerThread::~TrackerThread()
{
	m_mutex.Lock();
		m_quit = true;	// stop running loop
	m_mutex.Unlock();
	
	m_evCmd.Set();
	m_evData.Set();
	
	m_running.Wait();	// wait for running loop to stop
}

void TrackerThread::init(const Tracking::Tracker::Parameter &params)
{
	m_mutex.Lock();
		m_params = params;
		cmd = INIT;
	m_mutex.Unlock();
	m_evCmd.Set();
	m_evData.Wait(); m_evData.Reset();
}

int TrackerThread::addModelFromFile(const char* ply_file, TomGine::tgPose& pose, std::string label, bool bfc)
{
	int id;
	m_mutex.Lock();
		m_ply_file = std::string(ply_file);
		m_pose = pose;
		m_model_label = std::string(label);
		m_bfc = bfc;
		cmd = ADD_MODEL_FROM_FILE;
	m_mutex.Unlock();

	m_evCmd.Set();
	m_evData.Wait(); m_evData.Reset();
	
	m_mutex.Lock();
		id = m_model_id;
	m_mutex.Unlock();
	
	return id;
}

BOOL TrackerThread::OnTask()
{
	m_running.Lock();
	unsigned w,h;
	
	IplImage* image = 0;

	blortGLWindow::GLWindow* window = 0;
	Tracking::Tracker* tracker = 0;
	CameraThread* camera = 0;
	
	while(!m_quit)
	{
		switch(cmd)
		{
			case INIT:
				w = m_params.camPar.width;
				h = m_params.camPar.height;
				if(!camera){
					image = cvCreateImage( cvSize(w, h), 8, 3 );
					camera = new CameraThread(0, w, h);
					camera->SetThreadType(ThreadTypeIntervalDriven,0);
				}
				if(!window)
					window = new blortGLWindow::GLWindow(w, h, "Tracker");
				if(!tracker){
					tracker = new Tracking::TextureTracker();
					tracker->init(m_params);
				}
				m_mutex.Lock();
					m_evData.Set();
					cmd = IDLE;
				m_mutex.Unlock();
				break;
			case ADD_MODEL_FROM_FILE:
				if(tracker){
					m_model_id = tracker->addModelFromFile(m_ply_file.c_str(), m_pose, m_model_label.c_str(), m_bfc);
					m_mutex.Lock();
						m_evData.Set();
						cmd = TRACK;
					m_mutex.Unlock();
				}else{
					m_mutex.Lock();
						m_evData.Set();
						cmd = IDLE;
					m_mutex.Unlock();
				}
				break;
			case TRACK:
				if(tracker && camera && window && image){
					camera->GetImage(image);
					tracker->image_processing((unsigned char*)image->imageData);
					tracker->track();
					tracker->drawResult();
					tracker->drawCoordinates();
					window->Update();
				}
				break;
			case IDLE:
			default:
				m_evCmd.Wait(); m_evCmd.Reset();
				break;		
		}
		
	}
	
	if(camera) delete(camera);
	if(tracker) delete(tracker);
	if(window) delete(window);
	if(image) cvReleaseImage(&image);

	m_running.Unlock();
	return TRUE;
}




