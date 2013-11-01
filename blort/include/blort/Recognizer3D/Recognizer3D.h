 /**
 * @file Recognizer3D.h
 * @author Thomas Moerwald & Bence Magyar
 * @date April 2012
 * @version 0.2
 * @brief Main file of Recognizer for 3D pose estimation using SIFT features
 * @namespace blortRecognizer
 */

#ifndef _RECOGNIZER_3D_
#define _RECOGNIZER_3D_

#include <blort/Recognizer3D/DetectGPUSIFT.hh>
#include <blort/Recognizer3D/ODetect3D.hh>
#include <blort/Recognizer3D/Object3D.hh>
#include <blort/Recognizer3D/ModelObject3D.hh>
#include <blort/Recognizer3D/KeypointDescriptor.hh>
#include <blort/Recognizer3D/PoseCv.hh>
#include <blort/TomGine/tgModel.h>
#include <blort/TomGine/tgPose.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/CameraInfo.h>

namespace blortRecognizer{

/** @brief intrinsic camera parameter 
*   lookup OpenCV Camera Calibration and 3D Reconstruction
*   http://opencv.willowgarage.com/documentation/cpp/camera_calibration_and_3d_reconstruction.html */
struct CameraParameter {
	int w,h;
	float fx, fy;
	float cx, cy;
	float k1, k2, k3;
	float p1, p2;
        CameraParameter(){ w=h=fx=fy=cx=cy=k1=k2=k3=p1=p2=0; }
        CameraParameter(const sensor_msgs::CameraInfo& cam_info);
};

struct Siftex{
	vec3 pos;
	vec3 normal;
	vec3 viewray;
};

/** @brief Recognizer for 3D pose estimation using SIFT features */
class Recognizer3D{

public:
	/** @brief Construction of Recognizer
	*   @param camParam camera parameter for camera calibration
        *   @param prefix to be added to every file path
        *   @param display display sifts and found object in a window */
        Recognizer3D(const CameraParameter& camParam, std::string config_root = "",
                     bool display = false);
	~Recognizer3D();
	
	void setCameraParameter(const blortRecognizer::CameraParameter& camParam);
	
	void setGaussian(int kernel, float stdDeviation){ m_gauss_kernel = kernel; m_gauss_dev = stdDeviation; }
	
	/** @brief recognizes a object by using the loaded sift model file
	*   @param tFrame Image/Pixel map to search for the sift model
	*   @param pose returned pose of the object (if found)
	*   @param conf returned confidence of pose of the object
	*   @return true if object found, false if not */
	bool recognize(IplImage* tFrame, std::vector< boost::shared_ptr<TomGine::tgPose> > & poses, std::vector<float> & confs);
	
	/** @brief add sift features to sift model of an object
	*   @param tFrame image/pixel map to search for new sift features
	*   @param model shape description of model using faces and vertices (see TomGine::tgModel)
	*   @param pose pose of the model as seen in the image */
        bool learnSifts(IplImage* tFrame, const TomGine::tgModel& model, const TomGine::tgPose& pose);
	
	/** @brief load a sift model
	*   @param sift_file relative path and name to sift file (i.e.: "../Resources/sift/TeaBox.sift")
	*   @return success of loading the file */
        bool loadModelFromFile(const std::string sift_file);
	
	/** @brief save a sift model
	*   @param sift_file relative path and name to sift file (i.e.: "../Resources/sift/TeaBox.sift")
	*   @return success of saving the file */
	bool saveModelToFile(const char* sift_file);
	
	/** @brief get position and normal vector of sift features in model
	*   @param pl 3D point list of sift features (relative to object)
	*   @param nl 3D normal vector list of sift features (in object space) */
	void getSifts(std::vector<Siftex>& sl){ sl = m_siftexlist; }
	
	/** @brief get position and normal vector of sift features in model detected in the last call of learnSifts()
	*   @param pl 3D point list of sift features (relative to object)
	*   @param nl 3D normal vector list of sift features (in object space) */
	void getLastSifts(std::vector<Siftex> &sl){ sl = m_lastsiftexlist; }
	
        //BENCE
        cv::Mat getImage(){ return display_image; }
        void setDoUndistort(bool enable){ do_undistort = enable; }

        static void Convert(P::PoseCv& p1, TomGine::tgPose& p2);

        void setNNThreshold(double nn_threshold);
        void setRansacNPointsToMatch(unsigned int n);
        cv::Mat getDebugImage(){ return debug_image; }

private:
	Recognizer3D();
	
	P::DetectGPUSIFT sift;
    /* Used to load model in tracking, only one needed */
	P::ModelObject3D m_sift_model_learner;
    /* Use to perform detection, one should do it */
    P::ODetect3D m_detect;
    /* Store the model, need one per object */
	std::vector< boost::shared_ptr<P::Object3D> > m_sift_models;
	
    /* Store all sifts in the image, shared among the objects */
	P::Array<P::KeypointDescriptor*> m_image_keys;
	
	std::vector<Siftex> m_lastsiftexlist;
	std::vector<Siftex> m_siftexlist;
	
	CvMat *pIntrinsicDistort;
	CvMat *pDistortion;
	CvMat *C;
	CvMat *pMapX, *pMapY;
	CameraParameter m_cp;
	
	bool m_model_loaded;
	bool m_display;
	
	int m_gauss_kernel;
	float m_gauss_dev;

        //BENCE
        cv::Mat display_image;
        cv::Mat debug_image; // usually empty
        bool do_undistort;
        std::string config_root;
        //cv::Ptr<pal_blort::CvDetect3D> cv_detect;
};

}

#endif /* _RECOGNIZER_3D_ */
