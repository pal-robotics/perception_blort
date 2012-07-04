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
  * @date May 2012
  *
  * @brief This node converts geometry_msgs::Pose messages to tf transforms and publishes them. Has two command line arguments for parent_name and
  *        child_name, feel free to remap "pose". Example run: rosrun pal_vision_util pose2Tf stereo_optical_frame blort_target_frame
  *        pose:=blort_tracker/detection_result
  */


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <blort_ros/pose_util.h>
#include <string>

std::string parent_name;
std::string child_name;

void poseCallback(const geometry_msgs::Pose &msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform target_transform = pal_vision_util::rosPose2TfTransform(msg);
    br.sendTransform( tf::StampedTransform(target_transform, ros::Time::now(), parent_name, child_name));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pose2Tf");
    ros::NodeHandle nh;
    if(argc < 3)
    {
        ROS_ERROR("pose2Tf node requires a parent and a child name to publish to tf.");
        return -1;
    }

    parent_name = std::string(argv[1]);
    child_name = std::string(argv[2]);

    ros::Subscriber sub = nh.subscribe("/pose", 10, &poseCallback);
    ros::spin();
    return 0;
};
