

#ifndef _RECOGNIZER_THREAD_H_
#define _RECOGNIZER_THREAD_H_

#include <blort/ThreadObject/Thread.h>
#include <blort/Recognizer3D/Recognizer3D.h>
#include <vector>
#include <opencv2/core/core.hpp>

class CRecognizerThread : public CThread
{
private:
	enum Command{
		RECOGNIZE,
		LEARN,
		LOAD,
		SAVE,
		GETSIFT,
		GETLASTSIFT,
		IDLE,
	};
	Command cmd;
	
	CEventClass m_evCmd, m_evData;
	CMutexClass m_running;
	
	bool m_quit;
  std::map<std::string, double> m_confs;
	std::string m_sift_file;
	blortRecognizer::CameraParameter m_params;
  std::map<std::string, boost::shared_ptr<TomGine::tgPose> > m_poses;
	std::vector< boost::shared_ptr<TomGine::tgModel> > m_models;
	IplImage* m_image;
	std::vector<blortRecognizer::Siftex> m_lastsiftexlist;
	std::vector<blortRecognizer::Siftex> m_siftexlist;

  cv::Mat result;
  std::string config_root;
	
public:
        CRecognizerThread(const blortRecognizer::CameraParameter& params, std::string config_root="");
	~CRecognizerThread();
	
  void Recognize(IplImage* image, std::map<std::string, boost::shared_ptr<TomGine::tgPose> > & poses, std::map<std::string, double> & confs);
	
	void LearnSifts(IplImage* image,  TomGine::tgModel &model, TomGine::tgPose& pose);
	
  void LoadSiftModel(const std::string& sift_file);
	
  void SaveSiftModel(const std::string& sift_file);
	
	void GetSifts(std::vector<blortRecognizer::Siftex>& sl);
	
	void GetLastSifts(std::vector<blortRecognizer::Siftex>& sl);
	
	virtual BOOL OnTask();

  cv::Mat getImage(){ return result; }
};

#endif /* _RECOGNIZER_THREAD_H_ */

