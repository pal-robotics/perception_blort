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
 * @file gpugltracker.h
 * @author Bence Magyar
 * @date April 2012
 * @version 0.1
 * @brief The GpuGLTracker class encapsulates the Tracker module of BLORT.
 * It provides a C++ interface but uses ROS classes for data transfer at some places.
 * This class is a more civilized version of the mechanism that is in Projects/Learnsifts/main.cpp.
 */

#ifndef GPUGLTRACKER_H
#define GPUGLTRACKER_H

#include "trackerinterface.hpp"
#include <blort_ros/TrackerConfidences.h>
#include <blort_ros/TrackerCommand.h>
#include <blort_ros/TrackerConfig.h>
#include <geometry_msgs/Pose.h>
#include <string>

#include <blort/Tracker/TextureTracker.h>
#include <blort/TomGine/tgModel.h>
#include <blort/TomGine/tgTimer.h>

namespace blort_ros
{
    enum TrackerPublishMode
    {
        TRACKER_PUBLISH_GOOD = 0,
        TRACKER_PUBLISH_GOOD_AND_FAIR = 1,
        TRACKER_PUBLISH_ALL = 2
    };

    class GLTracker : public TrackerInterface
    {
    private:
        //config
        double conf_threshold;
        bool visualize_obj_pose;
        int publish_mode;

        float recovery_conf_threshold; // threshold used in recovery mode to say OK to a pose proposal

        // functionality
        TomGine::tgCamera::Parameter tgcam_params;
        TomGine::tgTimer timer;
        TomGine::tgPose cam_pose;
        Tracking::Tracker::Parameter track_params;

        Tracking::TextureTracker tracker;   // tracking module

        //config files //FIXME
        std::string config_root_, ply_model_;
        std::string model_name, sift_file; // name of the current model
        std::string pose_cal;   // filename with the pose calibration values

        // Model for Tracker
        TomGine::tgPose trPose; // current pose of the object used by the tracker module
        int model_id;
        Tracking::movement_state movement;
        Tracking::quality_state quality;
        Tracking::confidence_state tracker_confidence;

        // Initialise image
        IplImage *image; // iplimage object used be the former blort tracker module

        // result variables
        TrackerConfidences tracker_confidences;
        geometry_msgs::Pose fixed_cam_pose;
        std::vector<geometry_msgs::Pose> result;
        
        //reconf GUI hack
        bool last_reset;

    public:
        GLTracker(const sensor_msgs::CameraInfo camera_info,
                  const std::string& config_root,
                  bool visualize_obj_pose = false);

        /** @brief Method to run and handle recovery state. */
        virtual void recovery() {}

        /** @brief Method to run and handle tracking. */
        virtual void track();

        void reset();

        /** @brief Control the tracker using a ROS reconfigure_gui node.
         *  @param Reconfigure_gui messagetype */
        void reconfigure(blort_ros::TrackerConfig config);

        /** @brief Control the tracker with a single int code.
         *  @param code integer code associated with command, can be used with enums.
         *  @param param parameter of the command
         */
        void trackerControl(int code, int param = -1);

        void resetWithPose(const geometry_msgs::Pose& new_pose);

        /** @brief Get some statistics of the actual tracking state. */
        TrackerConfidences getConfidences(){ return tracker_confidences; }

        /** @brief Get the results of the latest detections. */
        const std::vector<geometry_msgs::Pose>& getDetections(){ return result; }

        /** @brief Get the constant camera reference of Blort. */
        const geometry_msgs::Pose getCameraReferencePose(){ return fixed_cam_pose; }

        /** @brief Get the rendered image for visualization. */
        cv::Mat getImage();

        /** @brief Get a status string describing the current state of the tracker. */
        std::string getStatusString();

        void setVisualizeObjPose(bool enable){ visualize_obj_pose = enable; }

        void setPublishMode(TrackerPublishMode mode){ publish_mode = mode; }

        TrackerPublishMode getPublishMode() { return (TrackerPublishMode)publish_mode; }

        void resetParticleFilter();

        ~GLTracker();

    private:
        /** @brief Update confidences and state based on the state and confidences of
          * the encapsulated tracker. Also pose result is updated if the state is apropriate.*/
        void update();

        /** @brief Assemble pose result to be published based on class variables.
          * The result is put in the corresponding variable. */
        void updatePoseResult();
    };
}

#endif // GPUGLTRACKER_H
