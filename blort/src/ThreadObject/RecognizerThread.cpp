
#include <blort/ThreadObject/RecognizerThread.h>
#include <stdexcept>
#include <opencv2/legacy/compat.hpp>

using namespace std;

CRecognizerThread::CRecognizerThread(const blortRecognizer::CameraParameter& params, std::string config_root)
{
	m_mutex.Lock();
		m_quit = false;
		m_params = params;
                this->config_root = config_root;
		cmd = IDLE;
		m_image = cvCreateImage( cvSize(params.w, params.h), 8, 3 );
	m_mutex.Unlock();

}

CRecognizerThread::~CRecognizerThread()
{
	m_mutex.Lock();
		m_quit = true;	// stop running loop
	m_mutex.Unlock();
	
	m_evCmd.Set();
	m_evData.Set();
	
	m_running.Wait();	// wait for running loop to stop
	
}

void CRecognizerThread::Recognize(IplImage* image, std::vector< boost::shared_ptr<TomGine::tgPose> > & poses, std::vector<float> & confs)
{
	m_mutex.Lock();
		cvCopyImage(image, m_image);
		m_poses = poses;
		cmd = RECOGNIZE;
	m_mutex.Unlock();
	
	m_evCmd.Set();
	m_evData.Wait(); m_evData.Reset();
	
	m_mutex.Lock();
		poses = m_poses;
		confs = m_confs;
	m_mutex.Unlock();
}

void CRecognizerThread::LearnSifts(IplImage* image,  TomGine::tgModel &model, TomGine::tgPose& pose)
{
	m_mutex.Lock();
		cvCopyImage(image, m_image);
        m_poses.clear();
        m_models.clear();
		m_poses.push_back(boost::shared_ptr<TomGine::tgPose>(new TomGine::tgPose(pose)));
		m_models.push_back(boost::shared_ptr<TomGine::tgModel>(new TomGine::tgModel(model)));
		cmd = LEARN;
	m_mutex.Unlock();
	
	m_evCmd.Set();
	m_evData.Wait(); m_evData.Reset();
}

void CRecognizerThread::LoadSiftModel(const std::string sift_file)
{
	m_mutex.Lock();
		m_sift_file = string(sift_file);
		cmd = LOAD;
	m_mutex.Unlock();
	
	m_evCmd.Set();
	m_evData.Wait(); m_evData.Reset();
}

void CRecognizerThread::SaveSiftModel(const std::string sift_file)
{
	m_mutex.Lock();
		m_sift_file = string(sift_file);
		cmd = SAVE;
	m_mutex.Unlock();
	
	m_evCmd.Set();
	m_evData.Wait(); m_evData.Reset();
}

void CRecognizerThread::GetSifts(std::vector<blortRecognizer::Siftex>& sl)
{
	m_mutex.Lock();
		cmd = GETSIFT;
	m_mutex.Unlock();
	
	m_evCmd.Set();
	m_evData.Wait(); m_evData.Reset();
	
	m_mutex.Lock();
		sl = m_siftexlist;
	m_mutex.Unlock();
}

void CRecognizerThread::GetLastSifts(std::vector<blortRecognizer::Siftex>& sl)
{
	m_mutex.Lock();
		cmd = GETLASTSIFT;
	m_mutex.Unlock();
	
	m_evCmd.Set();
	m_evData.Wait(); m_evData.Reset();
	
	m_mutex.Lock();
		sl = m_lastsiftexlist;
	m_mutex.Unlock();
}

BOOL CRecognizerThread::OnTask()
{
	m_running.Lock();
	
	
        blortRecognizer::Recognizer3D m_recognizer(m_params, config_root, true, true);
        //m_recognizer.setDoUndistort(false);
	
	while(!m_quit)
	{
		switch(cmd)
		{
			
			case RECOGNIZE:
				m_mutex.Lock();
					m_recognizer.recognize(m_image, m_poses, m_confs);
                                        result = m_recognizer.getImage();
					m_evData.Set();
					cmd = IDLE;
				m_mutex.Unlock();
				break;
			
			case LEARN:
				m_mutex.Lock();
					m_recognizer.learnSifts(m_image, *m_models[0], *m_poses[0]);
					m_evData.Set();
					cmd = IDLE;
				m_mutex.Unlock();
				break;
			
			case LOAD:
				m_mutex.Lock();
					m_recognizer.loadModelFromFile(m_sift_file.c_str());
					m_evData.Set();
					cmd = IDLE;
				m_mutex.Unlock();
				break;
			
			case SAVE:
				m_mutex.Lock();
					m_recognizer.saveModelToFile(m_sift_file.c_str());
					m_evData.Set();
					cmd = IDLE;
				m_mutex.Unlock();
				break;
				
			case GETSIFT:
				m_mutex.Lock();
					m_recognizer.getSifts(m_siftexlist);
					m_evData.Set();
					cmd = IDLE;
				m_mutex.Unlock();
				break;
				
			case GETLASTSIFT:
				m_mutex.Lock();
					m_recognizer.getLastSifts(m_lastsiftexlist);
					m_evData.Set();
					cmd = IDLE;
				m_mutex.Unlock();
				break;
			
			case IDLE:
			default:
				m_evCmd.Wait(); m_evCmd.Reset();
				break;		
		}
	}

	m_running.Unlock();
	return TRUE;
}




