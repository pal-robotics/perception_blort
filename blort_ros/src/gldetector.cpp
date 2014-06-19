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
 * @file gldetector.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief Class of GLDetector which wraps the detector core of BLORT.
*/

#include <blort_ros/gldetector.h>
#include <blort/Tracker/utilities.hpp>
#include <blort/TomGine/tgModelLoader.h>
#include <ros/console.h>
#include <sstream>
#include <iostream>
#include <blort/blort/pal_util.h>

using namespace blort_ros;

GLDetector::GLDetector(const sensor_msgs::CameraInfo& camera_info, const std::string& config_root)
{
  //this line should force opengl to run software rendering == no GPU
  //putenv("LIBGL_ALWAYS_INDIRECT=1");

  rec3dcounter = 0;
  recovery_conf_threshold = 0.05;

  //FIXME: make these ROS parameters or eliminate them and use the content as parameters
  std::string tracking_ini(blort_ros::addRoot("config/tracking.ini", config_root));
  std::vector<std::string> ply_models(0), sift_files(0), model_names(0);
  GetPlySiftFilenames(tracking_ini.c_str(), ply_models, sift_files, model_names);
  blort::buildFromFiles(ply_models, sift_files, model_names, objects, sift_index);

  recognizer = boost::shared_ptr<blortRecognizer::Recognizer3D>(
        new blortRecognizer::Recognizer3D(blortRecognizer::CameraParameter(camera_info), config_root, true));
  sift_files_count = 0;
  for(size_t i = 0; i < objects.size(); ++i)
  {
    for(size_t j = 0; j < objects[i].recog_data.size(); ++j)
    {
      recognizer->loadModelFromFile("Resources/sift/" + objects[i].recog_data[j].sift_file);
      sift_files_count++;
    }
  }
  image_ = cvCreateImage( cvSize(camera_info.width, camera_info.height), 8, 3 );
}

bool GLDetector::recovery(std::vector<std::string> & obj_ids, const cv::Mat& image,
                          blort_msgs::RecoveryCall::Response &resp)
{
  last_image = image;
  *image_ = last_image;

  return recoveryWithLast(obj_ids, resp);
}

bool GLDetector::recoveryWithLast(std::vector<std::string> & obj_ids,
                                  blort_msgs::RecoveryCall::Response &resp)
{
  double ticksBefore = cv::getTickCount();

  std::map< std::string, boost::shared_ptr<TomGine::tgPose> > recPoses;
  std::map<std::string, double> confs;
  std::map<std::string, bool> select;
  for(size_t i = 0; i < objects.size(); ++i)
  {
    for(size_t j = 0; j < objects[i].recog_data.size(); ++j)
    {
      recPoses[objects[i].recog_data[j].sift_file] =
          boost::shared_ptr<TomGine::tgPose>(new TomGine::tgPose());
      select[objects[i].recog_data[j].sift_file] = true;
    }
  }
  recognizer->recognize(image_, recPoses, confs, select);

  bool found_one = false;
  resp.object_founds.resize(obj_ids.size());
  resp.Poses.resize(obj_ids.size());
  for(size_t i = 0; i < objects.size(); ++i)
  {
    blort::RecogData recog = blort::getBest(objects[i]);
    ROS_INFO_STREAM("object (" << objects[i].name
                    << ") conf: " << confs[recog.sift_file]);
    // if the recovery's confidence is high enough then propose this new pose
    resp.object_founds[i] = ( confs[recog.sift_file] > recovery_conf_threshold );
    if(resp.object_founds[i])
    {
      found_one = true;
      resp.Poses[i] = blort_ros::tgPose2RosPose(*(recPoses[recog.sift_file]));
    }
  }
  ROS_WARN_STREAM("Tried to recover for the " << rec3dcounter++ << ". time.");
  ROS_INFO_STREAM("Recovery execution time: "
                  << 1000*(cv::getTickCount() - ticksBefore)/cv::getTickFrequency()
                  << " ms");
  return found_one;
}

void GLDetector::reconfigure(blort_ros::DetectorConfig config)
{
  recovery_conf_threshold = config.recovery_conf_threshold;
}

cv::Mat GLDetector::getImage()
{
  cv::Mat tmp;
  tmp = recognizer->getImage();
  return tmp.empty()?last_image:tmp; // do we need a copy?
}

cv::Mat GLDetector::getDebugImage()
{
  cv::Mat tmp;
  tmp = recognizer->getDebugImage();
  return tmp;
}

GLDetector::~GLDetector()
{
  cvReleaseImage(&image_);
}
