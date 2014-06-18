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

#include <blort_ros/gltracker.h>
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
    publish_mode = blort_ros::TRACKER_PUBLISH_GOOD_AND_FAIR;
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
    pose_cal = blort_ros::addRoot("config/pose.cal", config_root);
    //FIXME: make these ROS parameters or eliminate them and use the content as parameters
    std::string tracking_ini(blort_ros::addRoot("config/tracking.ini", config_root));

    std::vector<std::string> ply_models(0), sift_files(0), model_names(0);
    GetPlySiftFilenames(tracking_ini.c_str(), ply_models, sift_files, model_names);
    // Build ModelEntry with these entries
    blort::buildFromFiles(ply_models, sift_files, model_names, objects_);

    GetTrackingParameter(track_params, tracking_ini.c_str(), config_root);

    tgcam_params = TomGine::tgCamera::Parameter(camera_info);
    getCamPose(pose_cal.c_str(), cam_pose); // should get this from a TF call
    setCameraPose(tgcam_params, pose_cal.c_str()); // camPose and tgCamParams share a lot of stuff
    track_params.camPar = tgcam_params;

    tracker.init(track_params);

    for(size_t i = 0; i < objects_.size(); ++i)
    {
        trPoses[objects_[i].name] = boost::shared_ptr<TomGine::tgPose>(new TomGine::tgPose);
        trPoses[objects_[i].name]->t = vec3(0.0, 0.1, 0.0);
        trPoses[objects_[i].name]->Rotate(0.0f, 0.0f, 0.5f);
        model_ids[objects_[i].name] = tracker.addModelFromFile(blort_ros::addRoot(objects_[i].ply_model, config_root).c_str(), *trPoses[objects_[i].name], objects_[i].name.c_str(), true);
        movements[objects_[i].name] = Tracking::ST_SLOW;
        qualities[objects_[i].name] = Tracking::ST_LOST;
        tracking_confidences[objects_[i].name] = Tracking::ST_BAD;
        current_modes[objects_[i].name] = blort_ros::TRACKER_RECOVERY_MODE;
        current_confs[objects_[i].name] = blort_ros::TRACKER_CONF_LOST;
        //tracker_confidences.push_back(boost::shared_ptr<blort_msgs::TrackerConfidences>(new blort_msgs::TrackerConfidences));
        tracking_objects[objects_[i].name] = true;
    }
    //result.resize(objects_.size());
    tracker.setLockFlag(true);

    image = cvCreateImage( cvSize(tgcam_params.width, tgcam_params.height), 8, 3 );

    // define the constant cam_pose to be published
    fixed_cam_pose = blort_ros::tgPose2RosPose(cam_pose);
}

//2012-11-27: added by Jordi
void GLTracker::resetParticleFilter(std::string obj_i)
{
  //look up object based on name, then update --should be changed to nonbusy lookup
  BOOST_FOREACH(const blort::ObjectEntry& obj, objects_)
  {
    if(obj.name == obj_i)
    {
      tracker.removeModel(model_ids[obj_i]);
      model_ids[obj_i] = tracker.addModelFromFile(
            blort_ros::addRoot(obj.ply_model, config_root_).c_str(),
            *trPoses[obj_i], obj.name.c_str(), true);
      movements[obj_i] = Tracking::ST_SLOW;
      qualities[obj_i]  = Tracking::ST_LOST;
      tracking_confidences[obj_i] = Tracking::ST_BAD;
      tracking_objects[obj_i] = true;
	  break;
    }
  }
}

void GLTracker::track()
{
    boost::mutex::scoped_lock lock(models_mutex);
    *image = last_image;

    // Track object
    tracker.image_processing((unsigned char*)image->imageData);

    // turn on tracking for all selected objects
    BOOST_FOREACH(const blort::ObjectEntry& obj, objects_)
    {
      if(tracking_objects[obj.name])
        tracker.track(model_ids[obj.name]);
    }
    tracker.drawImage(0);
    tracker.drawCoordinates();
    BOOST_FOREACH(const blort::ObjectEntry& obj, objects_)
    {
        tracker.getModelPose(model_ids[obj.name], *trPoses[obj.name]);
    }
    tracker.drawResult(2.0f);

    // visualize current object pose if needed. moving this piece of code is troublesome,
    // has to stay right after drawImage(), because of OpenGL
    if( 1 || visualize_obj_pose)
    {
        for(size_t i = 0; i < trPoses.size(); ++i)
        {
            if(current_modes[objects_[i].name] == TRACKER_RECOVERY_MODE || !tracking_objects[objects_[i].name])
            {
                continue;
            }
            trPoses[objects_[i].name]->Activate();
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
            trPoses[objects_[i].name]->Deactivate();
        }
    }

    update();

    for(size_t i = 0; i < trPoses.size(); ++i)
    {
        if(qualities[objects_[i].name] == Tracking::ST_LOCKED)
        {
            this->current_modes[objects_[i].name] = blort_ros::TRACKER_LOCKED_MODE;
        }
        else if(qualities[objects_[i].name] == Tracking::ST_LOST)
        {
          ROS_INFO_STREAM("GLTracker::track: switching tracker to RECOVERY_MODE because object " << objects_[i].name << "LOST");
          switchToRecovery(objects_[i].name);
        }

        if(tracking_confidences[objects_[i].name] == Tracking::ST_GOOD
           && movements[objects_[i].name] == Tracking::ST_STILL
           && qualities[objects_[i].name] != Tracking::ST_LOCKED)
        {
          ROS_DEBUG_STREAM("Tracker is really confident (edge conf: "); //<< tracker_confidences[i]->edgeConf <<
                          //" lost conf: " << tracker_confidences[i]->lostConf << ". Sorry, no learning with this node.");
        }
    }
}

void GLTracker::resetWithPose(std::string id, const geometry_msgs::Pose& new_pose)
{
    boost::mutex::scoped_lock lock(models_mutex);
    ConvertCam2World(blort_ros::rosPose2TgPose(new_pose), cam_pose, *trPoses[id]);
    tracker.setModelInitialPose(model_ids[id], *trPoses[id]);
    //2012-11-28: commented by Jordi because the resetParticleFilter will reset the ModelEntry
    //tracker.resetUnlockLock(); // this does one run of the tracker to update the probabilities

    //2012-11-27: added by Jordi
    resetParticleFilter(id);

    switchToTracking(id);

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
        reset();
    }
}

void GLTracker::trackerControl(uint8_t code, const std::vector<std::string> & params)
{
    std::string param = "";
    if(params.empty())
    {
        param = objects_.begin()->name;
    }
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
      if ( param != "" )
          tracker.setModelModeFlag( model_ids[param] );
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
    case 8:
        switchTracking(params);
        break;
    }
}

void GLTracker::switchTracking(const std::vector<std::string> & params)
{
    if(params.size() == 0)
    {
        for(size_t i = 0; i < tracking_objects.size(); ++i)
        {
            tracking_objects[objects_[i].name] = !tracking_objects[objects_[i].name];
        }
    }
    else
    {
        for(size_t i = 0; i < params.size(); ++i)
        {
//            if(params[i] < tracking_objects.size()) // TODO: check if param is valid object name
//            {
                tracking_objects[params[i]] = !tracking_objects[params[i]];
//            }
        }
    }
}

cv::Mat GLTracker::getImage()
{
    cv::Mat tmp;

    // check if there is a different state then recovery mode
    blort_ros::tracker_mode current_mode = TRACKER_RECOVERY_MODE;
    typedef std::pair<std::string, blort_ros::tracker_mode> NameModePair_t;
    BOOST_FOREACH(NameModePair_t item, current_modes)
    {
        if(item.second != TRACKER_RECOVERY_MODE)
        {
            current_mode = item.second;
            break;
        }
    }
    switch(current_mode)
    {
    case TRACKER_RECOVERY_MODE:
        // here there is nothing new to publish, copy the last valid one
        return last_image.clone(); //TODO: do we need a copy?
        break;
    case TRACKER_TRACKING_MODE:
    case TRACKER_LOCKED_MODE:
        // get the last rendered image from the tracker
        return tracker.getImage().clone();
        break;
    default:
        break;
    }
    return tmp;
}

void GLTracker::updatePoseResult(std::string i)
{
    TomGine::tgPose pose;
    tracker.getModelPose(model_ids[i], pose);
    
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
    
    result[i] = detection;
}

void GLTracker::update()
{
    BOOST_FOREACH(blort::ObjectEntry& obj, objects_)
    {
        if(tracking_objects[obj.name])
        {
            //update confidences for output
            Tracking::ModelEntry* myModelEntry = tracker.getModelEntry(model_ids[obj.name]);
            obj.edgeConf = myModelEntry->c_edge;
            obj.confThreshold = myModelEntry->c_th;
            obj.lostConf = myModelEntry->c_lost;
            obj.distance = myModelEntry->t;

            //update confidences based on the currently tracked model
            // !!! the tracker state is now defined by the ONLY object tracked.
            // although the implementation would allow it, at several places, lacks this at several other.
            tracker.getModelMovementState(model_ids[obj.name], movements[obj.name]);
            tracker.getModelQualityState(model_ids[obj.name], qualities[obj.name]);
            ROS_INFO_STREAM("GLTracker::update: the tracked model for " << obj.name << " has set quality to " << qualities[obj.name]);
            tracker.getModelConfidenceState(model_ids[obj.name], tracking_confidences[obj.name]);

            switch(tracking_confidences[obj.name])
            {
            case Tracking::ST_GOOD:
                this->current_confs[obj.name] = blort_ros::TRACKER_CONF_GOOD;
                updatePoseResult(obj.name);
                break;
            case Tracking::ST_FAIR:
                this->current_confs[obj.name] = blort_ros::TRACKER_CONF_FAIR;
                if(publish_mode == TRACKER_PUBLISH_GOOD_AND_FAIR ||
                   publish_mode == TRACKER_PUBLISH_ALL)
                {
                    updatePoseResult(obj.name);
                    this->current_confs[obj.name] = blort_ros::TRACKER_CONF_GOOD;
                }
                break;
            case Tracking::ST_BAD:
                this->current_confs[obj.name] = blort_ros::TRACKER_CONF_FAIR;
                if(publish_mode == TRACKER_PUBLISH_ALL)
                    updatePoseResult(obj.name);
                break;
            default:
                ROS_ERROR("Unknown confidence value: %d", tracking_confidences[obj.name]);
                break;
            }
        }
    }
}

void GLTracker::reset(const std::vector<std::string> & params)
{
    ROS_INFO("GLTracker::reset: switching tracker to RECOVERY_MODE");
    if(!params.empty())
    {
        for(size_t i = 0; i < model_ids.size(); ++i)
        {
            switchToRecovery(objects_[i].name);
        }
    }
    else
    {
        for(size_t i = 0; i < params.size(); ++i)
        {
          //TODO: validate param
//            if(params[i] < model_ids.size())
                switchToRecovery(params[i]);
//            }
        }
    }
}

void GLTracker::switchToTracking(std::string id)
{
    TrackerInterface::switchToTracking(id);
    tracker.getModelEntry(model_ids[id])->st_quality = Tracking::ST_OK;
}

void GLTracker::switchToRecovery(std::string id)
{
    TrackerInterface::switchToRecovery(id);
    tracker.getModelEntry(model_ids[id])->st_quality = Tracking::ST_LOST;
}

GLTracker::~GLTracker()
{
}
