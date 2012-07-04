

#ifndef _CAMERA_THREAD_H_
#define _CAMERA_THREAD_H_

#include <blort/ThreadObject/Thread.h>
#include <blort/TomGine/tgTimer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/legacy/compat.hpp>

namespace Tracking{

class CameraThread : public CThread
{
private:
	bool m_new_image;
	int m_camID;
	int m_width;
	int m_height;
	
	TomGine::tgTimer m_timer;
	
	CvCapture* m_capture;
	IplImage* m_image;
		
	CEventClass m_evData;
	
public:
	CameraThread(int camID, int width, int height);
	~CameraThread();
	
	virtual BOOL OnTask();
	
	void GetImage(IplImage* image);
	
};

} // namespace Tracking

#endif /* _CAMERA_THREAD_H_ */

