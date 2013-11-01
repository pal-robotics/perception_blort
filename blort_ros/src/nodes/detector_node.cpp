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
#include "../gldetector.h"

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
    DetectorNode(std::string root = ".")
        : nh_("blort_detector"), it_(nh_), root_(root), detector(0)
    {
        cam_info_sub = nh_.subscribe("/blort_camera_info", 1, &DetectorNode::cam_info_callback, this);
        image_pub = it_.advertise("image_result", 1);
        //debug_pub = it_.advertise("image_debug", 1);
        cam_info_service = nh_.advertiseService("set_camera_info", &DetectorNode::setCameraInfoCb, this);
        counter = 0;

        nh_.param<double>("nn_match_threshold", nn_match_threshold, 0.65); //0.55
        nh_.param<int>("ransac_n_points_to_match", ransac_n_points_to_match, 4); //4

    }
    
    ~DetectorNode()
    {
        if(detector != 0)
            delete(detector);
    }

    bool recovery(blort_ros::RecoveryCall::Request &req,
                  blort_ros::RecoveryCall::Response &resp)
    {
        if(detector != 0)
        {
            ROS_INFO("Detector called the %u-th time.", ++counter);
            bool result;
            resp.object_id = req.object_id;
            if(!req.Image.data.empty())
            {
                cv_bridge::CvImagePtr cv_ptr;
                try
                {
                    //cv_ptr = cv_bridge::toCvCopy(req.Image, sensor_msgs::image_encodings::BGR8);
                  cv_ptr = cv_bridge::toCvCopy(req.Image, req.Image.encoding);
                  ROS_INFO_STREAM("\n\nENCODING = " << req.Image.encoding << "\n\n\n");
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return false;
                }
                result = detector->recovery(req.object_id.data, cv_ptr->image, resp);
            } else {
                ROS_INFO("Running detector on latest image.");
                result = detector->recoveryWithLast(req.object_id.data, resp);
            }
            cv_bridge::CvImage out_msg;
            out_msg.header = req.Image.header;
            out_msg.header.stamp = ros::Time::now();
            //out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image = detector->getImage();
            image_pub.publish(out_msg.toImageMsg());

//            cv_bridge::CvImage debug_msg;
//            debug_msg.header = req.Image.header;
//            debug_msg.header.stamp = ros::Time::now();
//            debug_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
//            debug_msg.image = detector->getDebugImage();
//            debug_pub.publish(debug_msg.toImageMsg());

            ROS_INFO("\n");
            ROS_INFO_STREAM("= > detector_node returned " << result << "\n");
            ROS_INFO("\n");

            return result;
        } else {
            ROS_ERROR("blort_detector service called while the core is uninitialized.");
            return false;
        }
    }

    bool setCameraInfoCb(blort_ros::SetCameraInfo::Request &req,
                         blort_ros::SetCameraInfo::Response &resp)
    {
        if(detector == 0)
            cam_info_callback(req.CameraInfo);
        return true;
    }

    void reconf_callback(blort_ros::DetectorConfig &config, uint32_t level)
    {
        if(detector != 0)
        {
            detector->reconfigure(config);
            ROS_INFO("Detector confidence threshold set to %f", config.recovery_conf_threshold);
        } else {
            ROS_WARN("Please publish camera_info for the tracker initialization.");
        }
    }
    
    void cam_info_callback(const sensor_msgs::CameraInfo &msg)
    {
        if(detector == 0)
        {
            ROS_INFO("Camera parameters received, ready to run.");
            cam_info_sub.shutdown();
            detector = new blort_ros::GLDetector(msg, root_);
            if(nn_match_threshold != 0.0)
                detector->setNNThreshold(nn_match_threshold);
            if(ransac_n_points_to_match != 0)
                detector->setRansacNPointsToMatch(ransac_n_points_to_match);
            pose_service = nh_.advertiseService("pose_service", &DetectorNode::recovery, this);
            // lines for dynamic_reconfigure
            server_ = std::auto_ptr<dynamic_reconfigure::Server<blort_ros::DetectorConfig> >
                      (new dynamic_reconfigure::Server<blort_ros::DetectorConfig>());
            f_ = boost::bind(&DetectorNode::reconf_callback, this, _1, _2);
            server_->setCallback(f_);
        }
    }
};

int main(int argc, char *argv[] )
{
    if(argc < 2)
    {
        ROS_ERROR("The first command line argument should be the package root!");
        return -1;
    }
    ros::init(argc, argv, "blort_detector");
    DetectorNode node(argv[1]);
    ros::spin();
    return 0;
}

