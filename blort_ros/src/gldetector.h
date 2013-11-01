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
 * @file gldetector.h
 * @author Bence Magyar
 * @date April 2012
 * @version 0.1
 * @brief The GLDetector class encapsulates the Recognizer module of BLORT.
 * It provides a C++ interface but uses ROS classes for data transfer at some places.
 */

#ifndef GLDetector_H
#define GLDetector_H

#include <geometry_msgs/Pose.h>
#include <string>
#include <blort_ros/RecoveryCall.h>

#include <blort/ThreadObject/RecognizerThread.h>
#include <blort/Recognizer3D/Recognizer3D.h>
#include <blort_ros/DetectorConfig.h>

namespace blort_ros
{
    class GLDetector
    {
    private:

        // EXPERIMENTAL temporary stuff
        int rec3dcounter;
        // end of EXPERIMENTAL

        float recovery_conf_threshold; // threshold used in recovery mode to say OK to a pose proposal

        // Create OpenGL Window and Tracker
        boost::shared_ptr<blortRecognizer::Recognizer3D> recognizer; // recovery component

        //config files //FIXME
        std::vector<std::string> model_names, sift_files; // name of the current model
        std::vector<std::string> pose_cals;   // filename with the pose calibration values

        // Initialise image
        IplImage *image_; // iplimage that used be the in former blort tracker module
        cv::Mat last_image;

        //reconf GUI hack
        bool last_reset;

    public:
        GLDetector(const sensor_msgs::CameraInfo& camera_info,
                   const std::string& config_root);

        /** @brief Method to run and handle recovery state. */
        bool recovery(size_t obj_id, const cv::Mat& image,
                      blort_ros::RecoveryCall::Response &resp);

        /** @brief Method to run on the previously stored image.
         *  @see recovery */
        bool recoveryWithLast(size_t obj_id, blort_ros::RecoveryCall::Response &resp);

        /** @brief Control the tracker using a ROS reconfigure_gui node.
         *  @param reconfigure_gui messagetype */
        void reconfigure(blort_ros::DetectorConfig config);

        /** @brief Get the rendered image for visualization. */
        cv::Mat getImage();

        /** @brief Set the threshold of the inner nearest neighbor match */
        void setNNThreshold(double nn_threshold)
        { recognizer->setNNThreshold(nn_threshold); }

        /** @brief Set the threshold of the inner nearest neighbor match */
        void setRansacNPointsToMatch(unsigned int n)
        { recognizer->setRansacNPointsToMatch(n); }

        ~GLDetector();

        cv::Mat getDebugImage();

    };
}

#endif // GLDetector_H
