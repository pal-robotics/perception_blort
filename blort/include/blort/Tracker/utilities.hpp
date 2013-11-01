#pragma once

#ifndef __BLORT_UTILITIES_HPP__
#define __BLORT_UTILITIES_HPP__

#include <blort/GLWindow/GLWindow.h>
#include <blort/Tracker/Tracker.h>
#include <blort/TomGine/tgCamera.h>
#include <blort/TomGine/tgMathlib.h>
#include <blort/Recognizer3D/Recognizer3D.h>
#include <blort/Tracker/CDataFile.h>
#include <stdexcept>
#include <string>
#include <blort/blort/pal_util.h>

// Converts pose from Camera coordinates to world space
void ConvertCam2World(const TomGine::tgPose& pC, const TomGine::tgPose& camPose, TomGine::tgPose& pW){

    pW = camPose.Transpose() * pC;
}

void ConvertCv2tgPoseWorld(	const cv::Mat_<double> &R, const cv::Mat_<double> &T, 
                                const TomGine::tgPose &camPose, TomGine::tgPose &pose)
{
    mat3 r;
    vec3 t, rvec;
    TomGine::tgPose pose_cam_coord, camPoseT;
    camPoseT = camPose.Transpose();

    t.x = T(0,0);
    t.y = T(1,0);
    t.z = T(2,0);
    rvec.x = R(0,0);
    rvec.y = R(1,0);
    rvec.z = R(2,0);
    r.fromRotVector(rvec);
    r = r.transpose();

    pose_cam_coord.SetPose(r,t);

    pose = camPoseT * pose_cam_coord;
}

void GetRecognizerParameter(blortRecognizer::CameraParameter& params, 
                            const TomGine::tgCamera::Parameter& camPar )
{
    params.w = camPar.width;
    params.h = camPar.height;
    params.fx = camPar.fx;
    params.fy = camPar.fy;
    params.cx = camPar.cx;
    params.cy = camPar.cy;
    params.k1 = camPar.k1;
    params.k2 = camPar.k2;
    params.k3 = camPar.k3;
    params.p1 = camPar.p1;
    params.p2 = camPar.p2;
}

void GetRecognizerParameter(	blortRecognizer::CameraParameter& params,
                                const char* cam_cal_file)
{
    CDataFile camCDF, poseCDF, iniCDF;

    // Load calibration and ini files
    if(!camCDF.Load(cam_cal_file)){
        char errmsg[128];
        sprintf(errmsg, "[utilities::GetTrackingParameter] Can not open cam_cal file '%s'", cam_cal_file);
        throw std::runtime_error(errmsg);
    }

    params.w = camCDF.GetInt("w");
    params.h = camCDF.GetInt("h");
    params.fx = camCDF.GetFloat("fx");
    params.fy = camCDF.GetFloat("fy");
    params.cx = camCDF.GetFloat("cx");
    params.cy = camCDF.GetFloat("cy");
    params.k1 = camCDF.GetFloat("k1");
    params.k2 = camCDF.GetFloat("k2");
    params.k3 = 0.0f;
    params.p1 = camCDF.GetFloat("p1");
    params.p2 = camCDF.GetFloat("p2");
}

void getCamPoseCvCheckerboard(const cv::Mat_<double> &R, const cv::Mat_<double> &T, TomGine::tgPose &camPose)
{
    vec3 r, t;
    mat3 Rm;
    cv::Mat_<double> cv_Rm = cv::Mat_<double> ( 3,3 );
    cv::Mat_<double> cv_rvec = cv::Mat_<double> ( 3,1 );
    cv::Mat_<double> cv_tvec = cv::Mat_<double> ( 3,1 );
    cv::Rodrigues(R, cv_Rm);
    cv::transpose(cv_Rm, cv_Rm);
    cv_tvec = -cv_Rm * T;				// t = -R^T * t + t
    cv::Rodrigues(cv_Rm, cv_rvec);

    r.x = cv_rvec(0,0);
    r.y = cv_rvec(1,0);
    r.z = cv_rvec(2,0);
    t.x = cv_tvec(0,0);
    t.y = cv_tvec(1,0);
    t.z = cv_tvec(2,0);
    Rm.fromRotVector(r);

    camPose.SetPose(Rm,t);
}

void getCamPose(const char* pose_cal_file, TomGine::tgPose& camPose){
    CDataFile cdfParams;
    if(!cdfParams.Load(pose_cal_file)){
        char errmsg[128];
        sprintf(errmsg, "[utilities::getCamPose] Can not open pose_cal file '%s'", pose_cal_file);
        throw std::runtime_error(errmsg);
    }

    vec3 t, r;
    mat3 R;

    std::string pose = cdfParams.GetString("pose");
    sscanf( pose.c_str(), "[%f %f %f] [%f %f %f]",
            &(t.x), &(t.y), &(t.z), &(r.x), &(r.y), &(r.z) );

    R.fromRotVector(r);

    camPose.SetPose(R,t);
}

void GetCameraParameterCV(  TomGine::tgCamera::Parameter& camPar, 
                            CvMat *pImgSize,
                            CvMat *pIntrinsicDistort,
                            CvMat *pDistortions,
                            const cv::Mat &R, const cv::Mat &T)
{

    if(pImgSize){
        camPar.width = cvmGet(pImgSize, 0,0);
        camPar.height = cvmGet(pImgSize, 1,0);
    }else{
        throw std::runtime_error("[utilities::GetCameraParameterCV] Wrong format for CvMat pImgSize");
    }

    if(pIntrinsicDistort){
        camPar.fx = cvmGet(pIntrinsicDistort, 0,0);
        camPar.fy = cvmGet(pIntrinsicDistort, 1,1);
        camPar.cx = cvmGet(pIntrinsicDistort, 0,2);
        camPar.cy = cvmGet(pIntrinsicDistort, 1,2);
    }else{
        throw std::runtime_error("[utilities::GetCameraParameterCV] Wrong format for CvMat pIntrinsicDistort");
    }

    if(pDistortions){
        camPar.k1 = cvmGet(pDistortions, 0,0);
        camPar.k2 = cvmGet(pDistortions, 1,0);
        camPar.k3 = cvmGet(pDistortions, 2,0);
        //camPar.k4 = cvmGet(pIntrinsicDistort, 3,0);
        camPar.p1 = 0.0f;
        camPar.p2 = 0.0f;
    }else{
        throw std::runtime_error("[utilities::GetCameraParameterCV] Wrong format for CvMat pDistortions");
    }

    camPar.zFar = 5.0f;
    camPar.zNear = 0.1f;

    // Invert pose of calibration pattern to get pose of camera
    vec3 r;
    cv::Mat Rm = cv::Mat( 3,3, CV_64F );
    cv::Mat rvec = cv::Mat( 3,1, CV_64F );
    cv::Mat tvec = cv::Mat( 3,1, CV_64F );

    cv::Rodrigues(R, Rm);

    cv::transpose(Rm, Rm);

    tvec = -Rm * T;				// t = -R^T * t

    cv::Rodrigues(Rm, rvec);

    r.x = (float)rvec.at<double>(0,0);
    r.y = (float)rvec.at<double>(1,0);
    r.z = (float)rvec.at<double>(2,0);
    camPar.pos.x = tvec.at<double>(0,0);
    camPar.pos.y = tvec.at<double>(1,0);
    camPar.pos.z = tvec.at<double>(2,0);

    camPar.rot.fromRotVector(r);
}

void setCameraPose(TomGine::tgCamera::Parameter& camPar, const char* pose_cal_file)
{
    CDataFile poseCDF;
    if(!poseCDF.Load(pose_cal_file)){
        char errmsg[128];
        sprintf(errmsg, "[utilities::GetTrackingParameter] Can not open pose_cal file '%s'", pose_cal_file);
        throw std::runtime_error(errmsg);
    }

    // Pose
    vec3 p, r;
    std::string pose = poseCDF.GetString("pose");
    sscanf( pose.c_str(), "[%f %f %f] [%f %f %f]", &(p.x), &(p.y), &(p.z), &(r.x), &(r.y), &(r.z) );
    //printf("%s\n", pose.c_str());
    //printf("%f %f %f, %f %f %f\n", p.x, p.y, p.z, r.x, r.y, r.z);
    camPar.pos.x = p.x;
    camPar.pos.y = p.y;
    camPar.pos.z = p.z;
    camPar.rot.fromRotVector(r);
}

void GetCameraParameter( TomGine::tgCamera::Parameter& camPar, const char* cam_cal_file, const char* pose_cal_file)
{
    CDataFile camCDF, poseCDF;

    // Load calibration and ini files
    if(!camCDF.Load(cam_cal_file)){
        char errmsg[128];
        sprintf(errmsg, "[utilities::GetTrackingParameter] Can not open cam_cal file '%s'", cam_cal_file);
        throw std::runtime_error(errmsg);
    }

    if(!poseCDF.Load(pose_cal_file)){
        char errmsg[128];
        sprintf(errmsg, "[utilities::GetTrackingParameter] Can not open pose_cal file '%s'", pose_cal_file);
        throw std::runtime_error(errmsg);
    }

    // Camera Parameters
    camPar.width = camCDF.GetInt("w");
    camPar.height = camCDF.GetInt("h");
    camPar.fx = camCDF.GetFloat("fx");
    camPar.fy = camCDF.GetFloat("fy");
    camPar.cx = camCDF.GetFloat("cx");
    camPar.cy = camCDF.GetFloat("cy");
    camPar.k1 = camCDF.GetFloat("k1");
    camPar.k2 = camCDF.GetFloat("k2");
    camPar.k3 = 0.0f;
    camPar.p1 = camCDF.GetFloat("p1");
    camPar.p2 = camCDF.GetFloat("p2");
    camPar.zFar = 5.0f;
    camPar.zNear = 0.1f;

    // Pose
    vec3 p, r;
    std::string pose = poseCDF.GetString("pose");
    sscanf( pose.c_str(), "[%f %f %f] [%f %f %f]", &(p.x), &(p.y), &(p.z), &(r.x), &(r.y), &(r.z) );
    //printf("%s\n", pose.c_str());
    //printf("%f %f %f, %f %f %f\n", p.x, p.y, p.z, r.x, r.y, r.z);
    camPar.pos.x = p.x;
    camPar.pos.y = p.y;
    camPar.pos.z = p.z;
    camPar.rot.fromRotVector(r);
}

void GetTrackingParameter( Tracking::Tracker::Parameter& params, const char* ini_file, std::string config_root = ".")
{
    CDataFile iniCDF;

    if(!iniCDF.Load(ini_file)){
        char errmsg[128];
        sprintf(errmsg, "[utilities::GetTrackingParameter] Can not open tracking_ini file '%s'", ini_file);
        throw std::runtime_error(errmsg);
    }

    // Tracking
    // Constraints
    params.variation.r.x = iniCDF.GetFloat("r.x", "Constraints") * Tracking::pi/180.0f;
    params.variation.r.y = iniCDF.GetFloat("r.y", "Constraints") * Tracking::pi/180.0f;
    params.variation.r.z = iniCDF.GetFloat("r.z", "Constraints") * Tracking::pi/180.0f;
    params.variation.t.x 	= iniCDF.GetFloat("t.x", "Constraints");
    params.variation.t.y 	= iniCDF.GetFloat("t.y", "Constraints");
    params.variation.t.z 	= iniCDF.GetFloat("t.z", "Constraints");
    params.variation.z 		= iniCDF.GetFloat("z", "Constraints");

    // Performance
    params.num_recursions = iniCDF.GetInt("recursions", "Performance");
    params.num_particles = iniCDF.GetInt("particles", "Performance");
    params.hypotheses_trials = iniCDF.GetInt("hypotheses", "Performance");
    params.convergence = iniCDF.GetInt("convergence", "Performance");

    // Resource Path
    params.modelPath = pal_blort::addRoot(iniCDF.GetString("ModelPath", "ResourcePath"), config_root);
    params.texturePath = pal_blort::addRoot(iniCDF.GetString("TexturePath", "ResourcePath"), config_root);
    params.shaderPath = pal_blort::addRoot(iniCDF.GetString("ShaderPath", "ResourcePath"), config_root);

    // Other
    params.edge_tolerance = iniCDF.GetFloat("EdgeMatchingTolerance", "Other") * Tracking::pi/180.0f;
    params.minTexGrabAngle = iniCDF.GetFloat("MinTextureGrabAngle", "Other") * Tracking::pi/180.0f;
    params.num_spreadings =  iniCDF.GetInt("NumberOfSpreadings", "Other");
    params.max_kernel_size = iniCDF.GetInt("MaxKernelSize", "Other");

    params.model_sobel_th = iniCDF.GetFloat("ModelSobelThreshold", "Other");
    params.image_sobel_th = iniCDF.GetFloat("ImageSobelThreshold", "Other");
    params.pred_no_convergence = iniCDF.GetFloat("PredictorNoConvergence", "Other");

    params.c_th_base = iniCDF.GetFloat("BaseThreshold", "Qualitative");
    params.c_th_min = iniCDF.GetFloat("MinThreshold", "Qualitative");
    params.c_th_fair = iniCDF.GetFloat("FairThreshold", "Qualitative");
    params.c_th_lost = iniCDF.GetFloat("LostThreshold", "Qualitative");

    params.c_mv_not = iniCDF.GetFloat("NoMovementThreshold", "Movement");
    params.c_mv_slow = iniCDF.GetFloat("SlowMovementThreshold", "Movement");
}

/*FIXME Left temporally for backward compatiblity */
void GetPlySiftFilenames(const char* ini_file, std::string &ply_file, std::string &sift_file, std::string &model_name)
{
    CDataFile iniCDF;

    if(!iniCDF.Load(ini_file)){
        char errmsg[128];
        sprintf(errmsg, "[utilities::GetTrackingParameter] Can not open tracking_ini file '%s'", ini_file);
        throw std::runtime_error(errmsg);
    }

    ply_file = iniCDF.GetString("ModelPath", "ResourcePath");
    ply_file.append(iniCDF.GetString("Model", "Files"));
    sift_file = iniCDF.GetString("SiftPath", "ResourcePath");
    sift_file.append(iniCDF.GetString("SiftModel", "Files"));

    model_name = iniCDF.GetString("Model", "Files");
    model_name = model_name.substr(0, model_name.find("."));

}

void GetPlySiftFilenames(const char* ini_file, std::vector<std::string> & ply_files, std::vector<std::string> & sift_files, std::vector<std::string> & model_names)
{
    CDataFile iniCDF;

    if(!iniCDF.Load(ini_file)){
        char errmsg[128];
        sprintf(errmsg, "[utilities::GetTrackingParameter] Can not open tracking_ini file '%s'", ini_file);
        throw std::runtime_error(errmsg);
    }

    ply_files.resize(0);
    sift_files.resize(0);
    model_names.resize(0);

    {
        std::string resource_path = iniCDF.GetString("ModelPath", "ResourcePath");
        std::stringstream ss;
        ss << iniCDF.GetString("Model", "Files");
        while(ss.good())
        {
            std::string ply_file;
            ss >> ply_file;
            ply_files.push_back(resource_path + ply_file);
            ply_file = ply_file.substr(0, ply_file.find("."));
            model_names.push_back(ply_file);
        }
    }
    {
        std::string resource_path = iniCDF.GetString("SiftPath", "ResourcePath");
        std::stringstream ss;
        ss << iniCDF.GetString("SiftModel", "Files");
        while(ss.good())
        {
            std::string sift_file;
            ss >> sift_file;
            sift_files.push_back(resource_path + sift_file);
        }
    }
}

bool InputControl(Tracking::Tracker* tracker, blortGLWindow::Event& event, std::string config_root =""){

    switch (event.type)
    {
    case blortGLWindow::TMGL_Press:
        switch (event.input)
        {
        case blortGLWindow::TMGL_Escape:
        case blortGLWindow::TMGL_q:
            return false;
            break;
        case blortGLWindow::TMGL_1: //1
            tracker->setKernelSize(0);
            printf("Kernel size: %d\n", (int)0);
            break;
        case blortGLWindow::TMGL_2: //2
            tracker->setKernelSize(1);
            printf("Kernel size: %d\n", (int)1);
            break;
        case blortGLWindow::TMGL_3: //3
            tracker->setKernelSize(2);
            printf("Kernel size: %d\n", (int)2);
            break;
        case blortGLWindow::TMGL_4: //4
            tracker->setEdgeShader();
            break;
        case blortGLWindow::TMGL_5: //5
            tracker->setColorShader();
            break;
        case blortGLWindow::TMGL_e: //e
            tracker->setEdgesImageFlag( !tracker->getEdgesImageFlag() );
            break;
        case blortGLWindow::TMGL_i: //i
            tracker->printStatistics();
            break;
        case blortGLWindow::TMGL_l: //l
            tracker->setLockFlag( !tracker->getLockFlag() );
            break;
        case blortGLWindow::TMGL_m: //m
            tracker->setModelModeFlag( tracker->getModelModeFlag()+1 );
            break;
        case blortGLWindow::TMGL_p: //p
            tracker->setDrawParticlesFlag( !tracker->getDrawParticlesFlag() );
            break;
        case blortGLWindow::TMGL_r: //r
            tracker->reset();
            break;
        case blortGLWindow::TMGL_s: //s
            tracker->saveModels(pal_blort::addRoot("Resources/ply/", config_root).c_str());
            break;
        case blortGLWindow::TMGL_t: //t
            tracker->textureFromImage(true);
            break;
        case blortGLWindow::TMGL_u: //u
            tracker->untextureModels();
            break;
        default:
            break;
        }
        break;
        case blortGLWindow::TMGL_None:
        break;
        case blortGLWindow::TMGL_Release:
        break;
        case blortGLWindow::TMGL_Motion:
        break;
        case blortGLWindow::TMGL_Expose:
        break;
        case blortGLWindow::TMGL_Quit:
        break;
    }
    return true;
}

CvMat* load_xml(const char* filename){
    char errmsg[128];
    CvMat *mat = (CvMat*)cvLoad(filename);
    if(mat==NULL){
        sprintf(errmsg, "[utilities::load_xml] cvLoad cannot open xml file '%s'\n", filename);
        throw std::runtime_error(errmsg);
    }
    return mat;
}



#endif // __BLORT_UTILITIES_HPP__
