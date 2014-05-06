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
 * @file pal_util.h
 * @author Bence Magyar
 * @date March 2012
 * @version 0.1
 * @brief This collection includes functions used for image processing
 * and various conversion functions used in BLORT.
 * @namespace pal_blort
 */

#ifndef PAL_UTIL_H
#define PAL_UTIL_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
#include <geometry_msgs/Pose.h>
#include "TomGine/tgPose.h"
#include <tf/transform_datatypes.h>

namespace pal_blort
{
    std::string addRoot(const std::string& obj, const std::string& root);

    geometry_msgs::Pose tgPose2RosPose(const TomGine::tgPose &pose);

    TomGine::tgPose rosPose2TgPose(const geometry_msgs::Pose &pose);

    tf::Transform rosPose2TfTransform(const geometry_msgs::Pose &pose);

    geometry_msgs::Pose tfTransform2RosPose(const tf::Transform &transform);

    geometry_msgs::Pose blortPosesToRosPose(geometry_msgs::Pose reference,
                                            geometry_msgs::Pose target);

    geometry_msgs::Pose poseDiff(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

    bool poseValidate(geometry_msgs::Pose known_pose,
                      geometry_msgs::Pose pose_estimate,
                      geometry_msgs::Pose max_error);

    cv::Mat quaternionTo3x3cvMat(geometry_msgs::Quaternion quaternion);
}

#endif // PAL_UTIL_H
