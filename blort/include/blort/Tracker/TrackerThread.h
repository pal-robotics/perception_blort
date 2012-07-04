

#ifndef _TRACKER_THREAD_H_
#define _TRACKER_THREAD_H_

#include <blort/ThreadObject/Thread.h>
#include <blort/Tracker/Tracker.h>
#include <blort/Tracker/CDataFile.h>
#include <blort/TomGine/tgPose.h>

namespace Tracking{

class TrackerThread : public CThread
{
private:
	enum Command{
		INIT,
		TRACK,
		ADD_MODEL_FROM_FILE,
		IDLE,
	};
	Command cmd;
	
	
	virtual BOOL OnTask();
	
	CEventClass m_evCmd, m_evData;
	CMutexClass m_running;
	
	bool m_quit;
	bool m_bfc;
	
	Tracking::Tracker::Parameter m_params;
	std::string m_tracking_ini;
	std::string m_ply_file;
	std::string m_model_label;
	int m_model_id;
	TomGine::tgPose m_pose;
	
public:
	TrackerThread();
	~TrackerThread();
	
	void init(const Tracking::Tracker::Parameter &params);

	int addModelFromFile(const char* ply_file, TomGine::tgPose& pose, std::string label, bool bfc=true);

};

} // namespace Tracking

#endif /* _TRACKER_THREAD_H_ */

