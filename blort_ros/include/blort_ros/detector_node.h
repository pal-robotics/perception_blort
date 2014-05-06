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
 * @file detector_node.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.2
 * @brief Main file of BLORT detector node for ROS.
 */

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <blort_ros/DetectorConfig.h>
#include <blort_ros/SetCameraInfo.h>
#include <blort_ros/gldetector.h>

class DetectorNode
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub;
    ros::Subscriber cam_info_sub;
    const std::string root_;
    ros::ServiceServer pose_service;
    ros::ServiceServer cam_info_service;
    std::auto_ptr<dynamic_reconfigure::Server<blort_ros::DetectorConfig> > server_;
    dynamic_reconfigure::Server<blort_ros::DetectorConfig>::CallbackType f_;
    blort_ros::GLDetector* detector;
    unsigned int counter;

    double nn_match_threshold;
    int ransac_n_points_to_match;

    //uncomment corresponding lines if need debug stuff
    //image_transport::Publisher debug_pub;

public:
    DetectorNode(std::string root = ".");

    ~DetectorNode();

    bool recovery(blort_ros::RecoveryCall::Request &req,
                  blort_ros::RecoveryCall::Response &resp);

    bool setCameraInfoCb(blort_ros::SetCameraInfo::Request &req,
                         blort_ros::SetCameraInfo::Response &resp);

    void reconf_callback(blort_ros::DetectorConfig &config, uint32_t level);
    
    void cam_info_callback(const sensor_msgs::CameraInfo &msg);
};

