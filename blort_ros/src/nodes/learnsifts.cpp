/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2012, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE. 
 *
 * @file learnsifts.cpp
 * @author Bence Magyar
 * @date April 2012
 * @version 0.2
 * @brief Main file of BLORT learnsifts node for ROS. Original source: Learnsifts.cpp of BLORT.
 */
#include <blort/Tracker/TextureTracker.h>
#include <blort/Tracker/utilities.hpp>
#include <blort/TomGine/tgModel.h>
#include <blort/TomGine/tgModelLoader.h>
#include <blort/TomGine/tgTimer.h>
#include <blort/GLWindow/GLWindow.h>
#include <blort/ThreadObject/RecognizerThread.h>

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <blort/blort/pal_util.h>

#include <blort_ros_msgs/TrackerConfidences.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>

cv::Mat lastImage;
bool need_new_image = true;
TomGine::tgCamera::Parameter tgCamParams;
bool need_cam_init;
ros::Subscriber cam_info_sub;
sensor_msgs::CameraInfo camera_info;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    lastImage = cv_ptr->image;
    need_new_image = false; //FIXME remove
}

void cam_info_callback(const sensor_msgs::CameraInfo &msg)
{
    tgCamParams = TomGine::tgCamera::Parameter(msg);
    ROS_INFO("Camera parameters received.");
    camera_info = msg;
    need_cam_init = false;
    cam_info_sub.shutdown();
}

struct TranslateStart
{
    int x;
    int y;
};

int main(int argc, char *argv[] )
{
    std::string config_root = argv[1];

    //this line should force opengl to run software rendering == no GPU
    //putenv("LIBGL_ALWAYS_INDIRECT=1");

    ros::init(argc, argv, "blort_learnsifts");
    ros::NodeHandle nh("blort_learnsifts");
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/blort_image", 1, &imageCb);
    cam_info_sub = nh.subscribe("/blort_camera", 1, &cam_info_callback);
    ros::Publisher confidences_pub = nh.advertise<blort_ros_msgs::TrackerConfidences>("confidences", 100);

    printf("\n Learnsifts \n\n");

    printf(" Tracker control\n");
    printf(" -------------------------------------------\n");
    printf(" [q,Esc] Quit program\n");
    printf(" [Return] Save SIFT and ply models.\n");
    printf(" [Space] Save texture face and extract SIFTS.\n");
    printf(" -------------------------------------------\n");
    printf(" [4] Texture edge tracking\n");
    printf(" [5] Texture color tracking\n");
    printf(" [a] High accuracy tracking (less robust)\n");
    printf(" [e] Show edges of image\n");
    printf(" [i] Print tracking information to console\n");
    printf(" [l] Lock/Unlock tracking\n");
    printf(" [m] Switch display mode of model\n");
    printf(" [p] Show particles\n");
    printf(" [r] Reset tracker to initial pose\n");
    printf(" [s] Save model to file (including textures)\n");
    printf(" [t] Capture model texture from image\n");
    printf(" [u] Untexture/remove texture from model\n");
    printf(" \n\n ");

    blortGLWindow::Event event;
    TomGine::tgTimer timer;
    std::vector<vec3> view_list;

    // File names
    //FIXME: make these ROS parameters or eliminate them and use the content as parameters
    std::string pose_cal = pal_blort::addRoot("config/pose.cal", config_root);
    std::string tracking_ini(pal_blort::addRoot("config/tracking.ini", config_root));
    std::vector<std::string> ply_models, sift_files, model_names;
    std::string ply_model, sift_file, model_name;

    GetPlySiftFilenames(tracking_ini.c_str(), ply_models, sift_files, model_names);
    ply_model = ply_models[0]; sift_file = sift_files[0]; model_name = model_names[0];
    printf("Object name: %s\n", model_name.c_str());
    sift_file = pal_blort::addRoot(sift_file, config_root);

    // Get Parameter from file
    TomGine::tgPose camPose, camPoseT;
    Tracking::Tracker::Parameter trackParams;
    GetTrackingParameter(trackParams, tracking_ini.c_str(), config_root);
    getCamPose(pose_cal.c_str(), camPose);
    camPoseT = camPose.Transpose();

    printf("=> Getting camera intrinsics ... ");
    // wait for the first camera_info message
    need_cam_init = true;
    while(need_cam_init)
    {
        ros::spinOnce();
    }
    setCameraPose(tgCamParams, pose_cal.c_str());
    trackParams.camPar = tgCamParams;

    printf("OK\n");

    // Initialise image
    IplImage *image = cvCreateImage( cvSize(camera_info.width, camera_info.height), 8, 3 ); //FIXME dirty

    printf("=> Initialising tracker ... ");

    // Create OpenGL Window and Tracker
    CRecognizerThread* pThreadRec =
            new CRecognizerThread(blortRecognizer::CameraParameter(camera_info));
    blortGLWindow::GLWindow window(trackParams.camPar.width, trackParams.camPar.height, "Training");
    Tracking::TextureTracker tracker;
    tracker.init(trackParams);
    float rec_time = 0.0f;
    printf("OK\n");

    // Model for Tracker
    TomGine::tgPose trPose;
    trPose.t = vec3(0.0, 0.1, 0.0);
    trPose.Rotate(0.0f, 0.0f, 0.5f);
    std::string modelFullPath = pal_blort::addRoot(ply_model, config_root);
    printf("=> Trying to get the object model from file: %s\n", modelFullPath.c_str());
    int modelID = tracker.addModelFromFile(modelFullPath.c_str(), trPose, model_name.c_str(), true);
    printf(" OK\n");
    tracker.setLockFlag(true);



    // Model for Recognizer / TomGine (ray-model intersection)
    TomGine::tgModel model;
    TomGine::tgPose tg_pose;
    TomGine::tgModelLoader modelloader;
    modelloader.LoadPly(model, pal_blort::addRoot(ply_model, config_root).c_str());

    Tracking::movement_state movement = Tracking::ST_FAST;
    Tracking::quality_state quality = Tracking::ST_OK;
    Tracking::confidence_state confidence = Tracking::ST_BAD;

    need_new_image = true;
    while(need_new_image)
    {
        ros::spinOnce();
    }
    *image = lastImage;

    pThreadRec->Event();

    bool quit = false;
    bool translateXY = false;
    bool translateZ = false;
    bool rotateXYZ = false;
    bool rotateX = false;
    bool rotateY = false;
    bool rotateZ = false;
    TranslateStart start;
    while(!quit)
    {
        tracker.getModelMovementState(modelID, movement);
        tracker.getModelQualityState(modelID, quality);
        tracker.getModelConfidenceState(modelID, confidence);

        if(confidence == Tracking::ST_GOOD && movement == Tracking::ST_STILL && quality != Tracking::ST_LOCKED)
        {
            ROS_WARN("Sorry, no learning with this node.");
            ROS_INFO("New initial pose fixed: \nt: [%f, %f, %f] \nq:[%f, %f, %f, %f]",
                     trPose.t.x,
                     trPose.t.y,
                     trPose.t.z,
                     trPose.q.x,
                     trPose.q.y,
                     trPose.q.z,
                     trPose.q.w);
        }

        //recovery, if object lost
        if(quality == Tracking::ST_LOST && rec_time > 0.2f)
        {
//            TomGine::tgPose recPose;
//            float conf;
//
//            pThreadRec->Recognize(image, recPose, conf);
//            if(conf > recovery_conf_threshold)
//            {
//                ConvertCam2World(recPose, camPose, trPose);
//                tracker.setModelInitialPose(modelID, trPose);
//                tracker.resetUnlockLock();
//            }
//            //tracker.reset();
//
//            ROS_INFO("orig conf: %f", conf);
//
//            // if the recovery's confidence is high enough then
//            // kick the tracker out of this state and let it try
//
//
//            rec_time = 0.0f;
//            ROS_WARN("Tried to recover for the %d. time.", ++rec3dcounter);
        }else{
            rec_time += timer.Update();
        }

        if(!need_new_image)
        {
            // Get image
            timer.Update();
            *image = lastImage;

            // Track object
            tracker.image_processing((unsigned char*)image->imageData);
            tracker.track();

            tracker.drawImage(0);
            tracker.drawCoordinates();
            tracker.getModelPose(modelID, trPose);
            tracker.drawResult(2.0f);
            tg_pose = trPose;
            need_new_image = true;
        }
        // Keyboard inputs:
        while(window.GetEvent(event)){
            quit = !InputControl(&tracker, event, config_root);
            if(event.type == blortGLWindow::TMGL_Press)
            {
                if(event.input == blortGLWindow::TMGL_Space)
                {
                    // When hitting space bar -> Learn new sift features
                    TomGine::tgPose pT1;
                    TomGine::tgPose pT2, pT3, pT4;
                    tracker.getModelPoseCamCoords(modelID, pT1);

                    mat3 R; vec3 a;
                    pT1.GetPose(R, a);
                    a = R.transpose() * pT1.t;
                    a.normalize();
                    view_list.push_back(a);

                    pT2 = pT1;
                    pThreadRec->LearnSifts(image, model, pT2);

                    std::vector<blortRecognizer::Siftex> sl;
                    pThreadRec->GetLastSifts(sl);
                    tracker.textureFromImage(true);
                }
                else if(event.input == blortGLWindow::TMGL_Return)
                {
                    // When hitting Return -> Save sift model and recognize
                    //TomGine::tgPose recPose;
                    //float conf;
                    pThreadRec->SaveSiftModel(sift_file.c_str());
                    //pThreadRec->Recognize(image, recPose, conf);
                    //ConvertCam2World(recPose, camPose, trPose);
                    //tracker.setModelInitialPose(modelID, trPose);
                    //tracker.reset(modelID);
                    tracker.saveModels(pal_blort::addRoot("Resources/ply/", config_root).c_str());
                }
            }
            if(event.type == blortGLWindow::TMGL_Press)
            {
                if( event.input == blortGLWindow::TMGL_Button3 && ! (translateZ || rotateXYZ) )
                {
                    translateXY = true;
                    start.x = event.motion.x; start.y = event.motion.y;
                }
                if( event.input == blortGLWindow::TMGL_Button2 && ! (translateXY || rotateXYZ) )
                {
                    translateZ = true;
                    start.x = event.motion.x; start.y = event.motion.y;
                }
                if( event.input == blortGLWindow::TMGL_x && ! (translateXY || translateZ || rotateXYZ) )
                {
                    std::cout << "Object rotation: (X) axis selected" << std::endl;
                    rotateX = true;
                    rotateY = false;
                    rotateZ = false;
                }
                if( event.input == blortGLWindow::TMGL_y && ! (translateXY || translateZ || rotateXYZ) )
                {
                    std::cout << "Object rotation: (Y) axis selected" << std::endl;
                    rotateX = false;
                    rotateY = true;
                    rotateZ = false;
                }
                if( event.input == blortGLWindow::TMGL_z && ! (translateXY || translateZ || rotateXYZ) )
                {
                    std::cout << "Object rotation: (Z) axis selected" << std::endl;
                    rotateX = false;
                    rotateY = false;
                    rotateZ = true;
                }
                if( event.input == blortGLWindow::TMGL_Button1 )
                {
                    rotateXYZ = true;
                    start.x = event.motion.x; start.y = event.motion.y;
                }
            }
            if(event.type == blortGLWindow::TMGL_Release)
            {
                if(event.input == blortGLWindow::TMGL_Button3)
                {
                    translateXY = false;
                }
                if(event.input == blortGLWindow::TMGL_Button2)
                {
                    translateZ = false;
                }
                if(event.input == blortGLWindow::TMGL_Button1)
                {
                    rotateXYZ = false;
                }
            }
            if(event.type == blortGLWindow::TMGL_Motion)
            {
                if(translateXY)
                {
                    float translateX = (start.x - event.motion.x)*0.001;
                    float translateY = (start.y - event.motion.y)*0.001;
                    trackParams.camPar.pos.x += trackParams.camPar.rot.mat[0]*translateX + trackParams.camPar.rot.mat[1]*translateY;
                    trackParams.camPar.pos.y += trackParams.camPar.rot.mat[3]*translateX + trackParams.camPar.rot.mat[4]*translateY;
                    trackParams.camPar.pos.z += trackParams.camPar.rot.mat[6]*translateX + trackParams.camPar.rot.mat[7]*translateY;
                    start.x = event.motion.x; start.y = event.motion.y;
                    tracker.setCameraParameters(trackParams.camPar);
                }
                if(translateZ)
                {
                    float translateZ = (start.y - event.motion.y)*0.001;
                    trackParams.camPar.pos.x += trackParams.camPar.rot.mat[2]*translateZ;
                    trackParams.camPar.pos.y += trackParams.camPar.rot.mat[5]*translateZ;
                    trackParams.camPar.pos.z += trackParams.camPar.rot.mat[8]*translateZ;
                    start.x = event.motion.x; start.y = event.motion.y;
                    tracker.setCameraParameters(trackParams.camPar);
                }
                if(rotateX && rotateXYZ)
                {
                    float thetaX = (-event.motion.x + start.x)*0.01;
                    TomGine::tgPose nPose;
                    tracker.getModelPose(modelID, nPose);
                    nPose.Rotate(thetaX, 0.0f, 0.0f);
                    tracker.setModelInitialPose(modelID, nPose);
                    tracker.reset(modelID);
                    start.x = event.motion.x; start.y = event.motion.y;
                }
                if(rotateY && rotateXYZ)
                {
                    float thetaY = (-event.motion.x + start.x)*0.01;
                    TomGine::tgPose nPose;
                    tracker.getModelPose(modelID, nPose);
                    nPose.Rotate(0.0f, thetaY, 0.0f);
                    tracker.setModelInitialPose(modelID, nPose);
                    tracker.reset(modelID);
                    start.x = event.motion.x; start.y = event.motion.y;
                }
                if(rotateZ && rotateXYZ)
                {
                    float thetaZ = (-event.motion.x + start.x)*0.01;
                    TomGine::tgPose nPose;
                    tracker.getModelPose(modelID, nPose);
                    nPose.Rotate(0.0f, 0.0f, thetaZ);
                    tracker.setModelInitialPose(modelID, nPose);
                    tracker.reset(modelID);
                    start.x = event.motion.x; start.y = event.motion.y;
                }
            }
            event.type = blortGLWindow::TMGL_None;
        }

        window.Update();

        //publish tracker inner confidences just for fun
        Tracking::ModelEntry* myModelEntry = tracker.getModelEntry(modelID);
        blort_ros_msgs::TrackerConfidences tr_conf;
        tr_conf.edgeConf = myModelEntry->c_edge;
        tr_conf.confThreshold = myModelEntry->c_th;
        tr_conf.lostConf = myModelEntry->c_lost;
        tr_conf.distance = myModelEntry->t;
	confidences_pub.publish(tr_conf);

        window.Activate();
        ros::spinOnce();
    }
    delete(pThreadRec);
    cvReleaseImage(&image);
}
