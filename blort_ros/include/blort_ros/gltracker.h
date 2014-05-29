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

#include <blort_ros/trackerinterface.hpp>
#include <blort_ros_msgs/TrackerConfidences.h>
#include <blort_ros_msgs/TrackerCommand.h>
#include <blort_ros/TrackerConfig.h>
#include <geometry_msgs/Pose.h>
#include <string>

#include <blort/Tracker/TextureTracker.h>
#include <blort/TomGine/tgModel.h>
#include <blort/TomGine/tgTimer.h>

#include "ObjectEntry.h"

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

        //config files
        std::string config_root_;
        std::vector<blort_ros::ObjectEntry> objects_;
        std::string pose_cal;   // filename with the pose calibration values

        // Model for Tracker
        std::vector< boost::shared_ptr<TomGine::tgPose> > trPoses; // current pose of the object used by the tracker module
        std::vector<int> model_ids;
        std::vector<Tracking::movement_state> movements;
        std::vector<Tracking::quality_state> qualities;
        std::vector<Tracking::confidence_state> tracking_confidences;
        std::vector<bool> tracking_objects;

        // Protection mutex for multi-threaded access to the model/poses
        boost::mutex models_mutex;

        // Initialise image
        IplImage *image; // iplimage object used be the former blort tracker module

        // result variables
        std::vector< boost::shared_ptr<blort_ros_msgs::TrackerConfidences> > tracker_confidences;
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

        void reset(const std::vector<uint8_t> & params = std::vector<uint8_t>(0));

        /** @brief Control the tracker using a ROS reconfigure_gui node.
         *  @param Reconfigure_gui messagetype */
        void reconfigure(blort_ros::TrackerConfig config);

        /** @brief Control the tracker with a single int code.
         *  @param code integer code associated with command, can be used with enums.
         *  @param param parameter of the command
         */
        void trackerControl(uint8_t code, const std::vector<uint8_t> & params);

        void resetWithPose(size_t obj_id, const geometry_msgs::Pose& new_pose);

        /** @brief Get some statistics of the actual tracking state. */
        const std::vector< boost::shared_ptr<blort_ros_msgs::TrackerConfidences> > & getConfidences(){ return tracker_confidences; }

        /** @brief Get the results of the latest detections. */
        const std::vector<geometry_msgs::Pose>& getDetections(){ return result; }

        /** @brief Get the constant camera reference of Blort. */
        const geometry_msgs::Pose getCameraReferencePose(){ return fixed_cam_pose; }

        /** @brief Get the rendered image for visualization. */
        cv::Mat getImage();

        /** @brief Return model names */
        const std::string & getModelName(size_t i) { return objects_[i].name; }

        void setVisualizeObjPose(bool enable){ visualize_obj_pose = enable; }

        void setPublishMode(TrackerPublishMode mode){ publish_mode = mode; }

        TrackerPublishMode getPublishMode() { return (TrackerPublishMode)publish_mode; }

        virtual void switchToTracking(size_t id);

        virtual void switchToRecovery(size_t id);

        virtual void switchTracking(const std::vector<uint8_t> & params);

        ~GLTracker();

    private:
        /** @brief Update confidences and state based on the state and confidences of
          * the encapsulated tracker. Also pose result is updated if the state is apropriate.*/
        void update();

        /** @brief Assemble pose result to be published based on class variables.
          * The result is put in the corresponding variable. */
        void updatePoseResult(size_t i);

        void resetParticleFilter(size_t id);
    };
}

#endif // GPUGLTRACKER_H
