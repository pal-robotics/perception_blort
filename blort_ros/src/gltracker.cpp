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
 * @file gltracker.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief Class of GLTracker which wraps the tracker core of BLORT.
 */

#include "gltracker.h"
#include <blort/Tracker/utilities.hpp>
#include <blort/TomGine/tgModelLoader.h>
#include <sstream>
#include <iostream>
#include <blort/blort/pal_util.h>
#include <ros/console.h>

using namespace blort_ros;

GLTracker::GLTracker(const sensor_msgs::CameraInfo camera_info,
                     const std::string& config_root,
                     bool visualize_obj_pose)
{
    //this line should force opengl to run software rendering == no GPU
    //putenv("LIBGL_ALWAYS_INDIRECT=1");
    config_root_ = config_root;
    conf_threshold = 0.4;
    recovery_conf_threshold = 0.05;
    publish_mode = 1;
    this->visualize_obj_pose = visualize_obj_pose;

    printf("\n Blort detection node\n\n");
    printf(" There is a dynamic_reconfigure interface for your convenience. \n");
    printf(" Control commands received via ROS service: tracker_control. \n");
    printf(" --------------------------\n");
    printf(" [0] Lock/Unlock tracking\n");
    printf(" [1] Reset tracker to initial pose\n");
    printf(" [2] Switch display mode of model\n");
    printf(" [3] Show particles\n");
    printf(" [4] Texture edge tracking\n");
    printf(" [5] Texture color tracking\n");
    printf(" [6] Show edges of image\n");
    printf(" [7] Print tracking information to console\n");
    printf(" \n\n ");

    // File names
    pose_cal = pal_blort::addRoot("bin/pose.cal", config_root);
    //FIXME: make these ROS parameters or eliminate them and use the content as parameters
    std::string tracking_ini(pal_blort::addRoot("bin/tracking.ini", config_root));
    std::string ply_model;

    GetPlySiftFilenames(tracking_ini.c_str(), ply_model, sift_file, model_name);
    ply_model_ = ply_model;
    GetTrackingParameter(track_params, tracking_ini.c_str(), config_root);

    tgcam_params = TomGine::tgCamera::Parameter(camera_info);
    getCamPose(pose_cal.c_str(), cam_pose); // should get this from a TF call
    setCameraPose(tgcam_params, pose_cal.c_str()); // camPose and tgCamParams share a lot of stuff
    track_params.camPar = tgcam_params;

    tracker.init(track_params);
    trPose.t = vec3(0.0, 0.1, 0.0);
    trPose.Rotate(0.0f, 0.0f, 0.5f);

    model_id = tracker.addModelFromFile(pal_blort::addRoot(ply_model, config_root).c_str(), trPose, model_name.c_str(), true);
    tracker.setLockFlag(true);

    image = cvCreateImage( cvSize(tgcam_params.width, tgcam_params.height), 8, 3 );

    movement = Tracking::ST_SLOW;
    quality = Tracking::ST_LOST;
    tracker_confidence = Tracking::ST_BAD;

    // define the constant cam_pose to be published
    fixed_cam_pose = pal_blort::tgPose2RosPose(cam_pose);
}

//2012-11-27: added by Jordi
void GLTracker::resetParticleFilter()
{
  tracker.removeModel(model_id);  
  model_id = tracker.addModelFromFile(pal_blort::addRoot(ply_model_, config_root_).c_str(), trPose, model_name.c_str(), true);
  movement = Tracking::ST_SLOW;
  quality  = Tracking::ST_LOST;
  tracker_confidence = Tracking::ST_BAD;  
}

void GLTracker::track()
{
    *image = last_image;

    // Track object
    tracker.image_processing((unsigned char*)image->imageData);

    ROS_INFO_STREAM("GLTracker::track: quality before TextureTracker::track is " << quality);
    tracker.track();
    ROS_INFO_STREAM("GLTracker::track: quality after TextureTracker::track is " << quality);

    tracker.drawImage(0);
    tracker.drawCoordinates();
    tracker.getModelPose(model_id, trPose);
    tracker.drawResult(2.0f);

    // visualize current object pose if needed. moving this piece of code is troublesome,
    // has to stay right after drawImage(), because of OpenGL
    if( 1 || visualize_obj_pose)
    {
        trPose.Activate();
        glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(1.0f, 0.0f, 0.0f);

        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 1.0f, 0.0f);

        glColor3f(0.0f, 0.0f, 1.0f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 1.0f);
        glEnd();
        trPose.Deactivate();
    }

    update();

    if(quality == Tracking::ST_LOCKED)
    {
        this->current_mode = blort_ros::TRACKER_LOCKED_MODE;
    }
    else if(quality == Tracking::ST_LOST)
    {
      ROS_INFO("GLTracker::track: switching tracker to RECOVERY_MODE because object LOST\n");
      switchToRecovery();
    }

    if(tracker_confidence == Tracking::ST_GOOD && movement == Tracking::ST_STILL && quality != Tracking::ST_LOCKED)
    {
      ROS_DEBUG_STREAM("Tracker is really confident (edge conf: " << tracker_confidences.edgeConf <<
                      " lost conf: " << tracker_confidences.lostConf << ". Sorry, no learning with this node.");
    }
}

void GLTracker::resetWithPose(const geometry_msgs::Pose& new_pose)
{
    ConvertCam2World(pal_blort::rosPose2TgPose(new_pose), cam_pose, trPose);
    tracker.setModelInitialPose(model_id, trPose);
    //2012-11-28: commented by Jordi because the resetParticleFilter will reset the ModelEntry
    //tracker.resetUnlockLock(); // this does one run of the tracker to update the probabilities

    //2012-11-27: added by Jordi
    resetParticleFilter();

    switchToTracking();

    //2012-11-27: commented by Jordi
    //update();

    // make sure that the tracker will start tracking after a recovery,
    // not standing still in locked mode
    tracker.setLockFlag(false);
}

void GLTracker::reconfigure(blort_ros::TrackerConfig config)
{
    tracker.setLockFlag(config.lock);
    tracker.setModelModeFlag(config.render_mode);
    tracker.setEdgesImageFlag(config.edge);
    visualize_obj_pose = config.visualize_obj_pose;
    publish_mode = config.publish_mode;

    //only reset the tracker when the checkbox was clicked, not depending on the act value
    if(last_reset != config.reset)
    {
        last_reset = config.reset;
        switchToRecovery();
    }
}

void GLTracker::trackerControl(int code, int param)
{
    switch(code)
    {
    case 0: //l
        tracker.setLockFlag( !tracker.getLockFlag() );
        break;
    case 1: //r
        //tracker.reset();
        this->reset();
        break;
    case 2: //m
        if ( param != -1 )
          tracker.setModelModeFlag( param );
        else
          tracker.setModelModeFlag( tracker.getModelModeFlag()+1 );
        break;
    case 3: //p
        tracker.setDrawParticlesFlag( !tracker.getDrawParticlesFlag() );
        break;
    case 4: //4
        tracker.setEdgeShader();
        break;
    case 5: //5
        tracker.setColorShader();
        break;
    case 6: //e
        tracker.setEdgesImageFlag( !tracker.getEdgesImageFlag() );
        break;
    case 7: //i
        tracker.printStatistics();
        break;
    }
}

std::string GLTracker::getStatusString()
{
    std::stringstream ss;
    ss << "GLTracker, mode: ";
    if(current_mode == blort_ros::TRACKER_TRACKING_MODE)
        ss << "tracking";
    else
        ss << "recovery";

    ss << " with target named: " << model_name << std::endl;
    return ss.str();
}

cv::Mat GLTracker::getImage()
{
    cv::Mat tmp;
    switch(current_mode)
    {
    case TRACKER_RECOVERY_MODE:
        return tmp.empty()?last_image.clone():tmp; // do we need a copy?
        break;
    case TRACKER_TRACKING_MODE:
        return tracker.getImage().clone();
        break;
    case TRACKER_LOCKED_MODE:
        return tracker.getImage().clone();
        break;
    }
    return tmp;
}

void GLTracker::updatePoseResult()
{
    result.clear();
    TomGine::tgPose pose;
    tracker.getModelPose(model_id, pose);

    geometry_msgs::Pose detection;
    detection.position.x = pose.t.x;
    detection.position.y = pose.t.y;
    detection.position.z = pose.t.z;
    //ALERT!!! inverse needed on the rotation quaternion because the blort orientation
    //output is computed differently than what rviz expects. they compute the inverse orientation
    detection.orientation.x = -pose.q.x;
    detection.orientation.y = -pose.q.y;
    detection.orientation.z = -pose.q.z;
    detection.orientation.w = pose.q.w;

    result.push_back(detection);
}

void GLTracker::update()
{
    //update confidences for output
    Tracking::ModelEntry* myModelEntry = tracker.getModelEntry(model_id);
    tracker_confidences.edgeConf = myModelEntry->c_edge;
    tracker_confidences.confThreshold = myModelEntry->c_th;
    tracker_confidences.lostConf = myModelEntry->c_lost;
    tracker_confidences.distance = myModelEntry->t;

    //update confidences based on the currently tracked model
    // !!! the tracker state is now defined by the ONLY object tracked.
    // although the implementation would allow it, at several places, lacks this at several other.
    tracker.getModelMovementState(model_id, movement);
    tracker.getModelQualityState(model_id, quality);
    ROS_INFO_STREAM("GLTracker::update: the tracked model has set quality to " << quality);
    tracker.getModelConfidenceState(model_id, tracker_confidence);

    switch(tracker_confidence)
    {
    case Tracking::ST_GOOD:
        this->current_conf = blort_ros::TRACKER_CONF_GOOD;
        updatePoseResult();
        break;
    case Tracking::ST_FAIR:      
        this->current_conf = blort_ros::TRACKER_CONF_FAIR;
        if(publish_mode == TRACKER_PUBLISH_GOOD_AND_FAIR ||
           publish_mode == TRACKER_PUBLISH_ALL)
        {
            updatePoseResult();
            this->current_conf = blort_ros::TRACKER_CONF_GOOD;
        }
        break;
    case Tracking::ST_BAD:
        this->current_conf = blort_ros::TRACKER_CONF_FAIR;
        if(publish_mode == TRACKER_PUBLISH_ALL)
            updatePoseResult();
        break;
    default:
        ROS_ERROR("Unknown confidence value: %d", tracker_confidence);
        break;
    }
}

void GLTracker::reset()
{
  ROS_INFO("GLTracker::reset: switching tracker to RECOVERY_MODE\n");
  switchToRecovery();
}

GLTracker::~GLTracker()
{
}
