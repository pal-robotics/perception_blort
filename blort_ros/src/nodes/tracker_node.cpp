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
 * @file tracker_node.cpp
 * @author Bence Magyar
 * @date May 2012
 * @version 0.2
 * @brief Main file of BLORT tracker node for ROS.
 */

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <ros/duration.h>
#include <ros/time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <dynamic_reconfigure/server.h>

#include <blort_ros/TrackerConfig.h>
#include <blort_ros/TrackerCommand.h>
#include <blort_ros/RecoveryCall.h>
#include <blort_ros/EstimatePose.h>
#include <blort_ros/SetCameraInfo.h>
#include <blort/GLWindow/glxhidingwindow.h>
#include <blort/blort/pal_util.h>
#include "../gltracker.h"
#include <boost/noncopyable.hpp>

class TrackerNode : boost::noncopyable
{
private:
    class Mode;
    class TrackingMode;
    class SingleShotMode;

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub;
    image_transport::Publisher image_debug_pub;
    ros::Publisher detection_result;
    ros::Publisher confidences_pub;

    //sensor_msgs::CameraInfo _msg;
    unsigned int pose_seq;
    std::string camera_frame_id;

    const std::string root_;
    ros::ServiceServer control_service;
    std::auto_ptr<dynamic_reconfigure::Server<blort_ros::TrackerConfig> > server_;
    dynamic_reconfigure::Server<blort_ros::TrackerConfig>::CallbackType f_;
    ros::ServiceClient recovery_client;
    blort_ros::GLTracker* tracker;
    std::string launch_mode;

    Mode* mode;
public:
    
    TrackerNode(std::string root = ".")
        : nh_("blort_tracker"), it_(nh_), pose_seq(0), camera_frame_id("0"), root_(root), tracker(0)
    {
        nh_.param<std::string>("launch_mode", launch_mode, "tracking");
        detection_result = nh_.advertise<geometry_msgs::PoseStamped>("detection_result", 100);
        confidences_pub = nh_.advertise<blort_ros::TrackerConfidences>("confidences", 100);
        image_pub = it_.advertise("image_result", 1);

        if(launch_mode == "tracking")
        {
            mode = new TrackingMode(this);
        }else if(launch_mode == "singleshot")
        {
            mode = new SingleShotMode(this);
        } else {
            ROS_FATAL("Invalid launch_mode parameter passed to blort_tracker.");
        }
    }
    
    ~TrackerNode()
    {
        if(tracker != 0)
            delete(tracker);
        delete mode;
    }

    void setCameraFrameID(const std::string & id)
    {
        camera_frame_id = id;
    }
    
    void imageCb(const sensor_msgs::ImageConstPtr& detectorImgMsg, const sensor_msgs::ImageConstPtr& trackerImgMsg )
    {
        if(tracker != 0)
        {
            cv_bridge::CvImagePtr cv_detector_ptr, cv_tracker_ptr;
            try
            {
                cv_detector_ptr = cv_bridge::toCvCopy(detectorImgMsg, sensor_msgs::image_encodings::BGR8);
                cv_tracker_ptr  = cv_bridge::toCvCopy(trackerImgMsg,  sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            if(tracker->getMode() == blort_ros::TRACKER_RECOVERY_MODE)
            {              
                blort_ros::RecoveryCall srv = mode->recovery(detectorImgMsg);

                ROS_INFO("tracker_node in TRACKER_RECOVERY_MODE: calling detector_node recovery service ...");
                if(recovery_client.call(srv))
                {
                  //tracker = new blort_ros::GLTracker(_msg, root_, true);
                  ROS_INFO("reseting tracker with pose from detector\n");
                  tracker->resetWithPose(srv.response.Pose);
                  ROS_INFO("AFTER reseting tracker with pose from detector\n");
                }
                else
                {
                    ROS_WARN("Detector not confident enough.\n");
                }
            }
            else //TRACKER_TRACKING_MODE or TRACKER_LOCKED_MODE
            {
              ROS_INFO("\n----------------------------------------------\n");
              ROS_INFO("TrackerNode::imageCb: calling tracker->process");
                tracker->process(cv_tracker_ptr->image);

                confidences_pub.publish(tracker->getConfidences());
                if(tracker->getConfidence() == blort_ros::TRACKER_CONF_GOOD ||
                    (tracker->getConfidence() == blort_ros::TRACKER_CONF_FAIR && tracker->getPublishMode() == blort_ros::TRACKER_PUBLISH_ALL) )
                {
                    geometry_msgs::PoseStamped target_pose;
                    target_pose.header.seq = pose_seq++;
                    target_pose.header.stamp = ros::Time::now();
                    target_pose.header.frame_id = camera_frame_id;
                    target_pose.pose = pal_blort::blortPosesToRosPose(tracker->getCameraReferencePose(),
                                                                                     tracker->getDetections()[0]);

                    detection_result.publish(target_pose);
                }

                cv_bridge::CvImage out_msg;
                out_msg.header = trackerImgMsg->header;
                out_msg.header.stamp = ros::Time::now();
                //out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = tracker->getImage();
                image_pub.publish(out_msg.toImageMsg());
            }
        }
    }
    
    bool trackerControlServiceCb(blort_ros::TrackerCommand::Request &req,
                                 blort_ros::TrackerCommand::Response &)
    {
        if(tracker != 0)
        {
            tracker->trackerControl(req.code, req.param);
            return true;
        } else {
            ROS_WARN("Please publish camera_info for the tracker initialization.");
            return false;
        }
    }

    // STATE DESIGN PATTERN
    // to implement the different tracker modes
private:
    class Mode
    {
    public:
        virtual void reconf_callback(blort_ros::TrackerConfig &config, uint32_t level) = 0;
        virtual blort_ros::RecoveryCall recovery(const sensor_msgs::ImageConstPtr& msg) = 0;
    };

    class TrackingMode : public Mode
    {
    private:
        ros::Subscriber cam_info_sub;        
        boost::shared_ptr<image_transport::SubscriberFilter> detector_image_sub, tracker_image_sub;
        boost::shared_ptr< message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image > > imageSynchronizer;
        TrackerNode* parent_;
    public:
        TrackingMode(TrackerNode* parent) : parent_(parent)
        {
            ROS_INFO("Blort tracker launched in tracking mode.");
            cam_info_sub = parent_->nh_.subscribe("/detector_camera_info", 10, &TrackerNode::TrackingMode::cam_info_callback, this);
        }

        virtual void reconf_callback(blort_ros::TrackerConfig &config, uint32_t level)
        {
            if(parent_->tracker != 0)
            {
                parent_->tracker->reconfigure(config);
            } else {
                ROS_WARN("Please publish camera_info for the tracker initialization.");
            }
        }

        virtual blort_ros::RecoveryCall recovery(const sensor_msgs::ImageConstPtr& msg)
        {
            blort_ros::RecoveryCall srv;
            srv.request.Image = *msg;
            return srv;
        }

        // The real initialization is being done after receiving the camerainfo.
        void cam_info_callback(const sensor_msgs::CameraInfo &msg)
        {
            if(parent_->tracker == 0)
            {
                ROS_INFO("Camera parameters received, ready to run.");
                parent_->setCameraFrameID(msg.header.frame_id);
                //parent_->_msg = msg; //2012-11-27 Jordi: keep a copy to reset the tracker when necessary
                cam_info_sub.shutdown();
                parent_->tracker = new blort_ros::GLTracker(msg, parent_->root_, true);
                parent_->tracker->setVisualizeObjPose(true);

                image_transport::TransportHints transportHint("raw");

                detector_image_sub.reset( new image_transport::SubscriberFilter(parent_->it_, "/detector_image",   1, transportHint) );
                tracker_image_sub.reset( new image_transport::SubscriberFilter(parent_->it_, "/tracker_image",   1, transportHint) );

                imageSynchronizer.reset( new message_filters::TimeSynchronizer<sensor_msgs::Image,
                                                                               sensor_msgs::Image>(*detector_image_sub, *tracker_image_sub, 1) );

                imageSynchronizer->registerCallback(boost::bind(&TrackerNode::imageCb, parent_, _1, _2));

                parent_->control_service = parent_->nh_.advertiseService("tracker_control", &TrackerNode::trackerControlServiceCb, parent_);
                parent_->server_ = std::auto_ptr<dynamic_reconfigure::Server<blort_ros::TrackerConfig> >
                                   (new dynamic_reconfigure::Server<blort_ros::TrackerConfig>());
                parent_->f_ = boost::bind(&TrackerNode::TrackingMode::reconf_callback, this, _1, _2);
                parent_->server_->setCallback(parent_->f_);
                parent_->recovery_client = parent_->nh_.serviceClient<blort_ros::RecoveryCall>("/blort_detector/pose_service");
            }
        }
    };

    class SingleShotMode : public Mode
    {
    private:
        ros::ServiceServer singleshot_service;
        double time_to_run_singleshot;
        ros::ServiceClient detector_set_caminfo_service;
        bool inServiceCall;
        TrackerNode* parent_;
        std::list<geometry_msgs::Pose> results;

        ros::Subscriber cam_info_sub;
        image_transport::Subscriber image_sub;
        sensor_msgs::ImageConstPtr lastImage;
        sensor_msgs::CameraInfoConstPtr lastCameraInfo;
    public:
        SingleShotMode(TrackerNode* parent) : parent_(parent)
        {
            ROS_INFO("Blort tracker launched in singleshot mode.");
            detector_set_caminfo_service = parent_->nh_.serviceClient<blort_ros::SetCameraInfo>("/blort_detector/set_camera_info");
            singleshot_service = parent_->nh_.advertiseService("singleshot_service", &TrackerNode::SingleShotMode::singleShotService, this);
            image_sub = parent_->it_.subscribe("/blort_image_rect_masked", 10, &TrackerNode::SingleShotMode::imageCallback, this);
            cam_info_sub = parent_->nh_.subscribe("/detector_camera_info", 10, &TrackerNode::SingleShotMode::cameraCallback, this);

            parent_->control_service = parent_->nh_.advertiseService("tracker_control", &TrackerNode::trackerControlServiceCb, parent_);
            parent_->server_ = std::auto_ptr<dynamic_reconfigure::Server<blort_ros::TrackerConfig> >
                               (new dynamic_reconfigure::Server<blort_ros::TrackerConfig>());
            parent_->f_ = boost::bind(&SingleShotMode::reconf_callback, this, _1, _2);
            parent_->server_->setCallback(parent_->f_);
            
            time_to_run_singleshot = 10.;
            inServiceCall = false;
        }

        void imageCallback(const sensor_msgs::ImageConstPtr &image)
        {
            if(!inServiceCall)
            {
                // if there is no ongoing servicecall, we can receive the new infos
                // if we are in an ongoing servicecall, these fields should be constant
                lastImage = image;
            }
        }

        void cameraCallback(const sensor_msgs::CameraInfoConstPtr &camera_info)
        {
            if(!inServiceCall)
            {
                // if there is no ongoing servicecall, we can receive the new infos
                // if we are in an ongoing servicecall, these fields should be constant
                lastCameraInfo = camera_info;
            }
        }

        virtual void reconf_callback(blort_ros::TrackerConfig &config, uint32_t level)
        {
            time_to_run_singleshot =  config.time_to_run_singleshot;
        }

        virtual blort_ros::RecoveryCall recovery(const sensor_msgs::ImageConstPtr& msg)
        {
            blort_ros::RecoveryCall srv;
            if(!inServiceCall)
            {
                srv.request.Image = *msg;
                // we step into the service call "state" with the first recoverycall made
                // no new images are accepted until the end of the singleShotServiceCall
                inServiceCall = true;
            }
            return srv;
        }

        bool singleShotService(blort_ros::EstimatePose::Request &req,
                               blort_ros::EstimatePose::Response &resp)
        {
            if(lastImage.use_count() < 1 && lastCameraInfo.use_count() < 1)
            {
                ROS_ERROR("Service called but there was no data on the input topics!");
                return false;
            } else {

                ROS_INFO("Singleshot service has been called with a timeout of %f seconds.", time_to_run_singleshot);
                results.clear();

                if(parent_->tracker == 0)
                {
                    parent_->tracker = new blort_ros::GLTracker(*lastCameraInfo, parent_->root_, true);
                    parent_->recovery_client = parent_->nh_.serviceClient<blort_ros::RecoveryCall>("/blort_detector/poseService");
                } else {
                    parent_->tracker->reset();
                }
                parent_->tracker->setPublishMode(blort_ros::TRACKER_PUBLISH_GOOD);
                parent_->tracker->setVisualizeObjPose(true);
                blort_ros::SetCameraInfo camera_info;
                camera_info.request.CameraInfo = *lastCameraInfo;
                if(!detector_set_caminfo_service.call(camera_info))
                    ROS_ERROR("blort_tracker failed to call blort_detector/set_camera_info service");

                double start_secs = ros::Time::now().toSec();
                while(ros::Time::now().toSec()-start_secs < time_to_run_singleshot)
                {
                    ROS_INFO("Remaining time %f", time_to_run_singleshot+start_secs-ros::Time::now().toSec());
                    parent_->imageCb(lastImage, lastImage);
                    if(parent_->tracker->getConfidence() == blort_ros::TRACKER_CONF_GOOD)
                    {
                        // instead of returning right away let's store the result
                        // to see if the tracker can get better
                        results.push_back(parent_->tracker->getDetections()[0]);
                    } else if(parent_->tracker->getConfidence() == blort_ros::TRACKER_CONF_LOST)
                    {
                        results.clear();
                    }
                }
                // we are out of the service call now, the results will be published
                inServiceCall = false;
                if(!results.empty())
                {
                    //convert results to a tf style transform and multiply them
                    //to get the camera-to-target transformation
                    resp.Pose = pal_blort::blortPosesToRosPose(parent_->tracker->getCameraReferencePose(),
                                                               results.back());
                    //NOTE: check the pose in vec3 location + mat3x3 rotation could be added here
                    // if we have any previous knowledge of the given scene
                    ROS_INFO_STREAM("PUBLISHED POSE: \n" << resp.Pose.position << "\n" <<
                                    pal_blort::quaternionTo3x3cvMat(resp.Pose.orientation) << "\n");
                    return true;
                } else {
                    //if the time was not enough to get a good detection, make the whole thing fail
                    return false;
                }
            }
        }
    };
};

int main(int argc, char *argv[] )
{
    if(argc < 2)
    {
        ROS_ERROR("The first command line argument should be the package root!");
        return -1;
    }
    ros::init(argc, argv, "blort_tracker");
    //FIXME: hardcoded size, 1x1 is not good, renders the tracker unfunctional in runtime
    // size should be not smaller the image size, too big size is also wrong
    pal_blort::GLXHidingWindow window(656, 492, "Tracker"); // a window which should hide itself after start
    //blortGLWindow::GLWindow window(640  , 480, "Window"); // a normal opengl window
    TrackerNode node(argv[1]);
    ros::spin();
    return 0;
}

