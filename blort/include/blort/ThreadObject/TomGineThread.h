

#ifndef _TOMGINE_THREAD_H_
#define _TOMGINE_THREAD_H_

// WIN32: Ensure that glew.h is included before gl.h
#ifdef WIN32
#include <GL/glew.h>
#endif

// WIN32: "ThreadObject/Thread.h" must be included before "Recognizer3D/Recognizer3D.h" and "TomGine/tgEngine.h"
#include <blort/ThreadObject/Thread.h>
// WIN32: "Recognizer3D/Recognizer3D.h" must be included before "TomGine/tgEngine.h"
#include <blort/Recognizer3D/Recognizer3D.h>
#include <blort/TomGine/tgEngine.h>

class CTomGineThread : public CThread
{
private:
	bool m_quit;
	
	bool m_use_campars;
	
	int m_width, m_height;
	std::vector<blortRecognizer::Siftex> m_siftexlist, m_lastsiftexlist;
	float m_max_vertex_length;
	TomGine::tgRenderModel m_model;
	TomGine::tgRenderModel m_sphere, m_cone;
	TomGine::tgPose m_pose;
	std::vector<TomGine::tgPose> m_viewlist;
	std::vector<vec3> m_viewscale;
	TomGine::tgCamera::Parameter m_camPars;
	
	CEventClass m_evCmd, m_evData;
	CMutexClass m_running;
	
	
public:
	CTomGineThread(int width, int height);
	CTomGineThread(int width, int height, const TomGine::tgCamera::Parameter& tgCamParams);
	~CTomGineThread();
	
	void SetModel(const TomGine::tgModel& model);
	void SetPose(const TomGine::tgPose& pose);
	void AddSifts(const std::vector<blortRecognizer::Siftex>& sl);
	void AddView(const TomGine::tgPose& view);
	
	virtual BOOL OnTask();
		
};

#endif /* _TOMGINE_THREAD_H_ */

