
#include <blort/Recognizer3D/Recognizer3D.h>

//#include <opencv/cv.h>
//#include <opencv/highgui.h>
//#include <opencv/cxcore.h>
#include <opencv2/core/core.hpp>

#include <blort/Recognizer3D/SDraw.hh>
#include <blort/Recognizer3D/cvgeometry.h>
#include <blort/TomGine/tgModelLoader.h>
#include <blort/TomGine/tgCollission.h>
#include <blort/blort/pal_util.h>
#include <ros/ros.h>

using namespace blortRecognizer;

CameraParameter::CameraParameter(const sensor_msgs::CameraInfo &cam_info)
{
    w = cam_info.width;
    h = cam_info.height;
    fx =cam_info.K.at(0);
    cx = cam_info.K.at(2);
    fy = cam_info.K.at(4);
    cy = cam_info.K.at(5);
    k1 = cam_info.D.at(0);
    k2 = cam_info.D.at(1);
    k3 = cam_info.D.at(4);
    p1 = cam_info.D.at(2);
    p2 = cam_info.D.at(3);
}

void Recognizer3D::Convert(P::PoseCv& p1, TomGine::tgPose& p2){

    mat3 cR;
    vec3 ct;

    cR[0] = cvmGet(p1.R,0,0); cR[3] = cvmGet(p1.R,0,1); cR[6] = cvmGet(p1.R,0,2);
    cR[1] = cvmGet(p1.R,1,0); cR[4] = cvmGet(p1.R,1,1); cR[7] = cvmGet(p1.R,1,2);
    cR[2] = cvmGet(p1.R,2,0); cR[5] = cvmGet(p1.R,2,1); cR[8] = cvmGet(p1.R,2,2);

    ct.x = cvmGet(p1.t,0,0);
    ct.y = cvmGet(p1.t,1,0);
    ct.z = cvmGet(p1.t,2,0);

    p2.SetPose(cR,ct);
}

Recognizer3D::Recognizer3D(const blortRecognizer::CameraParameter& camParam,
                           std::string config_root, bool display)
{
    do_undistort = true;
    this->config_root = config_root;
    pIntrinsicDistort = cvCreateMat(3,3, CV_64FC1);
    pDistortion = cvCreateMat(1,4, CV_64FC1);
    C = cvCreateMat(3,3, CV_32F);

    m_cp = camParam;

    cvmSet(pIntrinsicDistort, 0, 0, camParam.fx);
    cvmSet(pIntrinsicDistort, 0, 1, 0.);
    cvmSet(pIntrinsicDistort, 0, 2, camParam.cx);
    cvmSet(pIntrinsicDistort, 1, 0, 0.);
    cvmSet(pIntrinsicDistort, 1, 1, camParam.fy);
    cvmSet(pIntrinsicDistort, 1, 2, camParam.cy);
    cvmSet(pIntrinsicDistort, 2, 0, 0.);
    cvmSet(pIntrinsicDistort, 2, 1, 0.);
    cvmSet(pIntrinsicDistort, 2, 2, 1.);

    cvmSet(pDistortion, 0, 0, camParam.k1);
    cvmSet(pDistortion, 0, 1, camParam.k2);
    cvmSet(pDistortion, 0, 2, camParam.k3);
    cvmSet(pDistortion, 0, 3, 0.0);

    //camera matrix for undistorted images
    cvmSet(C, 0, 0, camParam.fx);
    cvmSet(C, 0, 1, 0.);
    cvmSet(C, 0, 2, camParam.cx);
    cvmSet(C, 1, 0, 0.);
    cvmSet(C, 1, 1, camParam.fy);
    cvmSet(C, 1, 2, camParam.cy);
    cvmSet(C, 2, 0, 0.);
    cvmSet(C, 2, 1, 0.);
    cvmSet(C, 2, 2, 1.);

    m_detect.SetCameraParameter(C);

    pMapX  = cvCreateMat (camParam.h,  camParam.w, CV_32FC1 );
    pMapY  = cvCreateMat (camParam.h,  camParam.w, CV_32FC1 );
    cvInitUndistortMapExact(pIntrinsicDistort, pDistortion, pMapX,  pMapY);

    m_gauss_kernel = 5;
    m_gauss_dev = 1.5;

    m_display = display;

    m_model_loaded = false;

    //    // create and initialize opencv-based 3D detector core
    //    cv_detect = cv::Ptr<pal_blort::CvDetect3D>(new pal_blort::CvDetect3D("FAST","SIFT","FlannBased"));
    //    cv_detect->setCameraMatrix(cv::Mat(pIntrinsicDistort, true));
    //    cv::Mat dist_coeffs(1, 5, CV_32FC1);
    //    dist_coeffs.at<float>(0,0) = camParam.k1;
    //    dist_coeffs.at<float>(0,1) = camParam.k2;
    //    dist_coeffs.at<float>(0,2) = camParam.p1;
    //    dist_coeffs.at<float>(0,3) = camParam.p2;
    //    dist_coeffs.at<float>(0,4) = camParam.k3;
    //    cv_detect->setDistCoeffs(dist_coeffs);
}

Recognizer3D::~Recognizer3D()
{
    cvReleaseMat(&pIntrinsicDistort);
    cvReleaseMat(&pDistortion);
    cvReleaseMat(&C);
    cvReleaseMat(&pMapX);
    cvReleaseMat(&pMapY);
}

void Recognizer3D::setCameraParameter(const blortRecognizer::CameraParameter& camParam)
{
    m_cp = camParam;

    cvmSet(pIntrinsicDistort, 0, 0, camParam.fx);
    cvmSet(pIntrinsicDistort, 0, 1, 0.);
    cvmSet(pIntrinsicDistort, 0, 2, camParam.cx);
    cvmSet(pIntrinsicDistort, 1, 0, 0.);
    cvmSet(pIntrinsicDistort, 1, 1, camParam.fy);
    cvmSet(pIntrinsicDistort, 1, 2, camParam.cy);
    cvmSet(pIntrinsicDistort, 2, 0, 0.);
    cvmSet(pIntrinsicDistort, 2, 1, 0.);
    cvmSet(pIntrinsicDistort, 2, 2, 1.);

    cvmSet(pDistortion, 0, 0, camParam.k1);
    cvmSet(pDistortion, 0, 1, camParam.k2);
    cvmSet(pDistortion, 0, 2, camParam.k3);
    cvmSet(pDistortion, 0, 3, 0.0);

    //camera matrix for undistorted images
    cvmSet(C, 0, 0, camParam.fx);
    cvmSet(C, 0, 1, 0.);
    cvmSet(C, 0, 2, camParam.cx);
    cvmSet(C, 1, 0, 0.);
    cvmSet(C, 1, 1, camParam.fy);
    cvmSet(C, 1, 2, camParam.cy);
    cvmSet(C, 2, 0, 0.);
    cvmSet(C, 2, 1, 0.);
    cvmSet(C, 2, 2, 1.);

    m_detect.SetCameraParameter(C);

    pMapX  = cvCreateMat (camParam.h,  camParam.w, CV_32FC1 );
    pMapY  = cvCreateMat (camParam.h,  camParam.w, CV_32FC1 );
    cvInitUndistortMapExact(pIntrinsicDistort, pDistortion, pMapX,  pMapY);
}

bool Recognizer3D::recognize(IplImage* tFrame, std::vector< boost::shared_ptr<TomGine::tgPose> > & poses, std::vector<float> & confs)
{
    P::PoseCv cvPose;
    bool detected = false;

    if(!m_model_loaded || m_sift_models[0]->codebook.Empty())
    {
        ROS_DEBUG("[Recognizer3D::recognize] Warning: no sift points in codebook of object (did you load the model before?) -- Possible GPU library error.");
        return false;
    }

    IplImage* tImg = 0;
    IplImage* grey = 0;

    tImg = cvCreateImage( cvGetSize ( tFrame ), 8, 3 );
    grey = cvCreateImage( cvGetSize ( tFrame ), 8, 1 );

    // undistort
    double ticksBefore = cv::getTickCount();
    if(do_undistort)
    {
        cvRemap(tFrame, tImg, pMapX, pMapY);
    }else{
        cvCopy(tFrame, tImg);
    }
    ROS_INFO("Recognizer3D::recognize: undistort time: %.01f ms", 1000*(cv::getTickCount() - ticksBefore)/cv::getTickFrequency());


    // convert to gray scale image
    cvConvertImage(tImg, grey);

    // extract sifts from current image
    ticksBefore = cv::getTickCount();
    sift.Operate(grey, m_image_keys);
    ROS_INFO("Recognizer3D::recognize: extract sift time: %.01f ms", 1000*(cv::getTickCount() - ticksBefore)/cv::getTickFrequency());

    m_detect.SetDebugImage(tImg);

    if(m_display)
    {
        sift.Draw(tImg, m_image_keys);        
    }

    //EXPERIMENTAL
    //cv_detect->detect(cv::Mat(grey, true), m_sift_model);
    ticksBefore = cv::getTickCount();
    for(size_t i = 0; i < m_sift_models.size(); ++i)
    {
        confs[i] = 0;
        bool detectResult = m_detect.Detect(m_image_keys, *m_sift_models[i]);
        ROS_INFO("Recognizer3D::recognize: ODetect3D::Detect time: %.01f ms\n", 1000*(cv::getTickCount() - ticksBefore)/cv::getTickFrequency());
        if ( detectResult )
        {
            if(m_sift_models[i]->conf > 0.03)
            {
                confs[i] = m_sift_models[i]->conf;
                if(m_display)
                {
                    // object contours & features drawn with green
                    P::SDraw::DrawPoly(tImg, m_sift_models[i]->contour.v, CV_RGB(0,255,0),2);
                    m_detect.DrawInlier(tImg, CV_RGB(0,255,0));
                }
                CopyPoseCv(m_sift_models[i]->pose, cvPose);
                Convert(cvPose, *poses[i]);
                ROS_DEBUG("[Recognizer3D::recognize] object found (conf: %f)\n", m_sift_models[i]->conf);
                detected = true;
            }else{
                if(m_display)
                    // pose estimage object contours drawn with purple
                    P::SDraw::DrawPoly(tImg, m_sift_models[i]->contour.v, CV_RGB(128,0,128),2);

                ROS_DEBUG("[Recognizer3D::recognize] No object (conf: %f)\n", m_sift_models[i]->conf);
                detected = false;
            }
	    
        }else{
            confs[i] = m_sift_models[i]->conf;

            if(m_display)
                P::SDraw::DrawPoly(tImg, m_sift_models[i]->contour.v, CV_RGB(128,0,128),2);

            ROS_DEBUG("[Recognizer3D::recognize] No object (conf: %f)\n", m_sift_models[i]->conf);
            detected = false;
        }
    }

    if(m_display)
    {
        display_image = cv::Mat(tImg, true);
    }

    cvReleaseImage(&tImg);
    cvReleaseImage(&grey);

    //EXPERIMENTAL
    //debug_image = cv_detect->getImage();

    return detected;
}

bool Recognizer3D::learnSifts(IplImage* tFrame, const TomGine::tgModel& m1, const TomGine::tgPose& pose)
{	
    P::Array<P::KeypointDescriptor*> m_tmp_keys;

    mat3 R, Rt;
    vec3 t;
    TomGine::tgModel m2 = m1;
    m_lastsiftexlist.clear();

    // Transform model vertices to camera coordinats
    pose.GetPose(R,t);
    Rt = R.transpose();
    for(unsigned int i=0; i<m2.m_vertices.size(); i++)
    {
        m2.m_vertices[i].pos = (R * m2.m_vertices[i].pos) + t;
        m2.m_vertices[i].normal = R * m2.m_vertices[i].normal;
    }

    IplImage* tImg = 0;
    IplImage* grey = 0;
    float dcpfx = 1/m_cp.fx;
    float dcpfy = 1/m_cp.fy;

    tImg = cvCreateImage ( cvGetSize ( tFrame ), 8, 3 );
    grey = cvCreateImage ( cvGetSize ( tFrame ), 8, 1 );

    // undistort
    cvRemap(tFrame, tImg, pMapX, pMapY );

    // convert to gray scale image
    cvConvertImage(tImg, grey);

    //EXPERIMENTAL:
    //    cv::Mat descriptors;
    //    std::vector<cv::KeyPoint> keypoints;
    //    cv_detect->extract(cv::Mat(grey, true), descriptors, keypoints);
    //
    //    pal_blort::CvDetect3D::cvKeypoints2BlortKeyPoints(
    //            descriptors, keypoints, m_image_keys);
    // END OF EXPERIMENTAL

    sift.Operate(grey, m_image_keys);

    for(unsigned int i=0; i<m_image_keys.Size(); i++)
    {
        vec3 point, normal;
        std::vector<vec3> pl;		// point list
        std::vector<vec3> nl;		// normal list
        std::vector<double> zl; // z-value list (depth)
        float u,v,x,y;

        u = (float)m_image_keys[i]->p.x;
        v = (float)m_image_keys[i]->p.y;
        x = (u-m_cp.cx) * dcpfx;
        y = (v-m_cp.cy) * dcpfy;

        // Create ray from pixel
        TomGine::tgRay ray;
        ray.start = vec3(0.0f,0.0f,0.0f);
        ray.dir =  vec3(x,y,1.0f);
        ray.dir.normalize();

        if(	TomGine::tgCollission::IntersectRayModel(pl, nl, zl, ray, m2)
            && !pl.empty()
            && !zl.empty())
            {
            // determine nearest intersection point
            unsigned int idx_min = 0;
            float z_min = zl[0];
            for(unsigned int idx=0; idx<zl.size(); idx++){
                if(zl[idx] < z_min){
                    idx_min = idx;
                    z_min = zl[idx];
                }
            }

            if(z_min > 0.0f){
                // Transform to object space
                point = (Rt * (pl[idx_min] - t));
                normal = (Rt * nl[idx_min]);
                vec3 r = (Rt * ray.dir) * zl[idx_min];

                Siftex s;
                s.pos = point;
                s.normal = normal;
                s.viewray = r;

                normal.normalize();
                r.normalize();

                if( (r * normal) < -0.1f)
                {
                    // Save sift
                    m_siftexlist.push_back(s);
                    m_lastsiftexlist.push_back(s);
                    m_image_keys[i]->SetPos(point.x, point.y, point.z);
                    m_tmp_keys.PushBack(m_image_keys[i]);
                }
            }
        }

    }

    if(m_tmp_keys.Size() > 0)
    {
        m_sift_model_learner.AddToModel(m_tmp_keys, *m_sift_models[0]);
        ROS_INFO("[Recognizer3D::learnSifts] added %d sifts to model\n", m_tmp_keys.Size());
        m_model_loaded = true;
    }

    if(m_display){
        display_image = cv::Mat(tImg, true);
    }

    cvReleaseImage(&tImg);
    cvReleaseImage(&grey);

    return true;
}

bool Recognizer3D::loadModelFromFile(const std::string sift_file)
{
    ROS_INFO("[Recognizer3D::loadModelFromFile] loading sift model from '%s'\n", sift_file.c_str());
    m_sift_models.push_back(boost::shared_ptr<P::Object3D>(new P::Object3D()));
    m_model_loaded = m_sift_model_learner.LoadModel(pal_blort::addRoot(sift_file, config_root), *m_sift_models[m_sift_models.size()-1]);
    //EXPERIMENTAL
    //cv_detect->addCodeBook(m_sift_model);
    return true;
}

bool Recognizer3D::saveModelToFile(const char* sift_file)
{
    ROS_INFO("[Recognizer3D::saveModelToFile] saving sift model to '%s'\n", sift_file);
    m_sift_model_learner.SaveModel(pal_blort::addRoot(sift_file, config_root).c_str(), *m_sift_models[0]);
    return true;
}

void Recognizer3D::setNNThreshold(double nn_threshold)
{
    m_detect.setNNThreshold(nn_threshold);
}

void Recognizer3D::setRansacNPointsToMatch(unsigned int n)
{
    m_detect.setNPointsToMatch(n);
}
