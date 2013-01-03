

#ifndef _CAMERA_THREAD_H_
#define _CAMERA_THREAD_H_

#include <blort/ThreadObject/Thread.h>
#include <blort/TomGine/tgTimer.h>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>

class CCameraThread : public CThread
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
	CCameraThread(int camID, int width, int height);
	~CCameraThread();
	
	virtual BOOL OnTask();
	
	bool GetImage(IplImage* image);
	void GetSize(int &width, int &height){ width = m_width; height = m_height; }
	
};

#endif /* _CAMERA_THREAD_H_ */

