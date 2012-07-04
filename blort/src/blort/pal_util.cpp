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
 * @file pal_util.cpp
 * @author Bence Magyar
 * @date April 2012
 * @version 0.1
 * @brief Util functions used in BLORT.
 */

#include <blort/blort/pal_util.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <sstream>
#include <iostream>
#include <algorithm>

namespace pal_blort
{
    //FIXME: check 'root' and put ./ instead of / to the beginning of the paths if it's empty
    std::string addRoot(const std::string& obj, const std::string& root)
    {
        std::stringstream ss;
        ss << root << "/" << obj;
        return ss.str();
    }

    geometry_msgs::Pose tgPose2RosPose(const TomGine::tgPose &pose)
    {
        geometry_msgs::Pose res;
        res.position.x = pose.t.x;
        res.position.y = pose.t.y;
        res.position.z = pose.t.z;
        res.orientation.x = pose.q.x;
        res.orientation.y = pose.q.y;
        res.orientation.z = pose.q.z;
        res.orientation.w = pose.q.w;
        return res;
    }

    TomGine::tgPose rosPose2TgPose(const geometry_msgs::Pose &pose)
    {
        TomGine::tgPose res;
        res.t.x = pose.position.x;
        res.t.y = pose.position.y;
        res.t.z = pose.position.z;
        res.q.x = pose.orientation.x;
        res.q.y = pose.orientation.y;
        res.q.z = pose.orientation.z;
        res.q.w = pose.orientation.w;
        return res;
    }

    tf::Transform rosPose2TfTransform(const geometry_msgs::Pose &pose)
    {
        tf::Transform result;
        result.setOrigin(tf::Vector3(pose.position.x,
                                     pose.position.y,
                                     pose.position.z));
        result.setRotation(tf::Quaternion(pose.orientation.x,
                                          pose.orientation.y,
                                          pose.orientation.z,
                                          pose.orientation.w
                                          ));
        return result;
    }

    geometry_msgs::Pose tfTransform2RosPose(const tf::Transform &transform)
    {
        geometry_msgs::Pose result;
        const tf::Vector3 position = transform.getOrigin();
        const tf::Quaternion orientation = transform.getRotation();
        result.position.x = position.x();
        result.position.y = position.y();
        result.position.z = position.z();
        result.orientation.x = orientation.x();
        result.orientation.y = orientation.y();
        result.orientation.z = orientation.z();
        result.orientation.w = orientation.w();

        return result;
    }

    geometry_msgs::Pose blortPosesToRosPose(geometry_msgs::Pose reference,
                                            geometry_msgs::Pose target)
    {
        const tf::Transform blort_reference_frame =
                pal_blort::rosPose2TfTransform(reference);
        const tf::Transform blort_target_pose =
                pal_blort::rosPose2TfTransform(target);

        return pal_blort::tfTransform2RosPose(blort_reference_frame.inverse()*blort_target_pose);
    }

    geometry_msgs::Pose poseAbsDiff(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
    {
        geometry_msgs::Pose result;
        result.position.x = abs(pose1.position.x-pose2.position.x);
        result.position.y = abs(pose1.position.y-pose2.position.y);
        result.position.z = abs(pose1.position.z-pose2.position.z);
        result.orientation.x = abs(pose1.orientation.x-pose2.orientation.x);
        result.orientation.y = abs(pose1.orientation.y-pose2.orientation.y);
        result.orientation.z = abs(pose1.orientation.z-pose2.orientation.z);
        result.orientation.w = abs(pose1.orientation.w-pose2.orientation.w);
        return result;
    }

    bool poseValidate(geometry_msgs::Pose known_pose,
                      geometry_msgs::Pose pose_estimate,
                      geometry_msgs::Pose max_error)
    {
        const geometry_msgs::Pose diff = poseAbsDiff(pose_estimate, known_pose);
        if((diff.position.x - max_error.position.x < 0) &&
           (diff.position.y - max_error.position.y < 0) &&
           (diff.position.z - max_error.position.z < 0) &&
           (diff.orientation.x - max_error.orientation.x < 0) &&
           (diff.orientation.y - max_error.orientation.y < 0) &&
           (diff.orientation.z - max_error.orientation.z < 0) &&
           (diff.orientation.w - max_error.orientation.w < 0))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    cv::Mat quaternionTo3x3cvMat(geometry_msgs::Quaternion quaternion)
    {
        float x2 = quaternion.x * quaternion.x;
        float y2 = quaternion.y * quaternion.y;
        float z2 = quaternion.z * quaternion.z;
        float xy = quaternion.x * quaternion.y;
        float xz = quaternion.x * quaternion.z;
        float yz = quaternion.y * quaternion.z;
        float wx = quaternion.w * quaternion.x;
        float wy = quaternion.w * quaternion.y;
        float wz = quaternion.w * quaternion.z;

        cv::Mat result(3,3, CV_32FC1);
        result.at<float>(0,0) = 1.0f - 2.0f * (y2 + z2);
        result.at<float>(0,1) = 2.0f * (xy - wz);
        result.at<float>(0,2) = 2.0f * (xz + wy);

        result.at<float>(1,0) = 2.0f * (xy + wz);
        result.at<float>(1,1) = 1.0f - 2.0f * (x2 + z2);
        result.at<float>(1,2) = 2.0f * (yz - wx);

        result.at<float>(2,0) = 2.0f * (xz - wy);
        result.at<float>(2,1) = 2.0f * (yz + wx);
        result.at<float>(2,2) = 1.0f - 2.0f * (x2 + y2);

        return result;
    }
}
