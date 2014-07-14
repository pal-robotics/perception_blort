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

#ifndef _H_TRACKER_NODE_H_
#define _H_TRACKER_NODE_H_

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
#include <actionlib/server/action_server.h>

#include <blort_ros/TrackerConfig.h>
#include <blort_msgs/TrackerResults.h>
#include <blort_msgs/TrackerCommand.h>
#include <blort_msgs/TrackerConfidences.h>
#include <blort_msgs/RecoveryCall.h>
#include <blort_msgs/EstimatePose.h>
#include <blort_msgs/SetCameraInfo.h>
#include <blort_msgs/RecognizeAction.h>
#include <blort/GLWindow/glxhidingwindow.h>
#include <blort/blort/pal_util.h>
#include <blort_ros/gltracker.h>
#include <boost/noncopyable.hpp>
#include <boost/thread.hpp>


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

  boost::mutex recovery_mutex;
  boost::thread recovery_th;
  std::map<std::string, geometry_msgs::Pose> recovery_answers;

  Mode* mode;
public:

  TrackerNode(std::string root = ".");

  ~TrackerNode();

  void setCameraFrameID(const std::string & id);

  void imageCb(const sensor_msgs::ImageConstPtr& detectorImgMsg,
               const sensor_msgs::ImageConstPtr& trackerImgMsg );

  bool trackerControlServiceCb(blort_msgs::TrackerCommand::Request &req,
                               blort_msgs::TrackerCommand::Response &);

private:

  void recovery(blort_msgs::RecoveryCall srv);

  // STATE DESIGN PATTERN
  // to implement the different tracker modes
private:
  class Mode
  {
  public:
    virtual void reconf_callback(blort_ros::TrackerConfig &config, uint32_t level) = 0;
    virtual blort_msgs::RecoveryCall getRecoveryCall(std::vector<std::string> & i,
                                                     const sensor_msgs::ImageConstPtr& msg) = 0;
  };

  class TrackingMode : public Mode
  {
  private:
    ros::Subscriber cam_info_sub;
    boost::shared_ptr<image_transport::SubscriberFilter> detector_image_sub, tracker_image_sub;
    boost::shared_ptr< message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image > > imageSynchronizer;
    TrackerNode* parent_;
  public:
    TrackingMode(TrackerNode* parent);

    virtual void reconf_callback(blort_ros::TrackerConfig &config, uint32_t level);

    virtual blort_msgs::RecoveryCall getRecoveryCall(std::vector<std::string> & i,
                                                     const sensor_msgs::ImageConstPtr& msg);

    // The real initialization is being done after receiving the camerainfo.
    void cam_info_callback(const sensor_msgs::CameraInfo &msg);
  };

  class SingleShotMode : public Mode
  {
    typedef actionlib::ActionServer<blort_msgs::RecognizeAction> AcServer;
  private:
    ros::ServiceServer singleshot_service;
    AcServer as_;
    double time_to_run_singleshot;
    ros::ServiceClient detector_set_caminfo_service;
    bool inServiceCall;
    double conf_treshold_;
    TrackerNode* parent_;
    std::vector<geometry_msgs::Pose> results_list;

    sensor_msgs::ImageConstPtr lastImage;
    sensor_msgs::CameraInfoConstPtr lastCameraInfo;

    blort_msgs::RecognizeFeedback feedback_;
    blort_msgs::RecognizeResult result_;

    double getDistance(const sensor_msgs::ImageConstPtr& img, double x, double y, double z);

  public:
    SingleShotMode(TrackerNode* parent);

    void imageCallback(const sensor_msgs::ImageConstPtr &image);

    void cameraCallback(const sensor_msgs::CameraInfoConstPtr &camera_info);

    virtual void reconf_callback(blort_ros::TrackerConfig &config, uint32_t level);

    virtual blort_msgs::RecoveryCall getRecoveryCall(std::vector<std::string> & i,
                                                     const sensor_msgs::ImageConstPtr& msg);

    /* FIXME Implement single-shot with object selection */
    /* For now, single-shot runs on first object for backward compatibility */
    bool singleShotService(blort_msgs::EstimatePose::Request &req,
                           blort_msgs::EstimatePose::Response &resp);

    void goalCb(AcServer::GoalHandle gh);
  };
};

#endif // ifndef _H_TRACKER_NODE_H_

