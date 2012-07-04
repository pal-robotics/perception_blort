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
 */

/** 
  * @author Bence Magyar
  */

#include <blort_ros/pose_util.h>

namespace pal_vision_util
{
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
                pal_vision_util::rosPose2TfTransform(reference);
        const tf::Transform blort_target_pose =
                pal_vision_util::rosPose2TfTransform(target);

        return pal_vision_util::tfTransform2RosPose(blort_reference_frame.inverse()*blort_target_pose);
    }
}
