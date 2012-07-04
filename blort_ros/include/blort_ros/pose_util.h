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

/** \author Bence Magyar */


#ifndef _PAL_POSE_UTIL_H_
#define _PAL_POSE_UTIL_H_

#include <vector>
#include <string>
#include <tf/transform_datatypes.h>

namespace pal_vision_util
{
    /**
      * @brief converts from geometry_msgs::Pose to tf::Transform
      */
    tf::Transform rosPose2TfTransform(const geometry_msgs::Pose &pose);

   
    /**
      * @brief converts from tf::Transform to geometry_msgs::Pose
      */
    geometry_msgs::Pose tfTransform2RosPose(const tf::Transform &transform);

    /**
      * @brief pal_blort specific helper function. Given the pose of the BLORT reference
      *        frame and the tracked object frame it returns a geometry_msgs::Pose corresponding 
      *        to the pose of the tracked object in the camera frame
      *
      * @param[in] reference pose of the BLORT reference frame with respect to the camera frame
      * @param[in] target pose of the tracked object frame with respect to the BLORT reference frame
      * @return the pose of the object frame with respect to the camera frame
      */
    geometry_msgs::Pose blortPosesToRosPose(geometry_msgs::Pose reference,
                                            geometry_msgs::Pose target);
}

#endif // _PAL_POSE_UTIL_H_
