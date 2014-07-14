#include <blort_ros/tracker_node.h>
#include <boost/foreach.hpp>
#include <sstream>

TrackerNode::TrackerNode(std::string root)
  : nh_("blort_tracker"), it_(nh_), pose_seq(0), camera_frame_id("0"), root_(root), tracker(0)
{
  nh_.param<std::string>("launch_mode", launch_mode, "tracking");
  detection_result = nh_.advertise<blort_msgs::TrackerResults>("detection_result", 100);
  confidences_pub = nh_.advertise<blort_msgs::TrackerConfidences>("confidences", 100);
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

TrackerNode::~TrackerNode()
{
  if(tracker != 0)
    delete(tracker);
  delete mode;
}

void TrackerNode::setCameraFrameID(const std::string & id)
{
  camera_frame_id = id;
}

void TrackerNode::imageCb(const sensor_msgs::ImageConstPtr& detectorImgMsg,
                          const sensor_msgs::ImageConstPtr& trackerImgMsg )
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

    bool should_process = false;
    std::vector<std::string> lost_ids;
    BOOST_FOREACH(const blort::ObjectEntry& obj, tracker->getObjects())
    {
      if(obj.is_tracked)
      {
        if(tracker->getMode(obj.name) == blort_ros::TRACKER_RECOVERY_MODE)
        {
          if(recovery_answers.count(obj.name))
          {
            tracker->resetWithPose(obj.name, recovery_answers[obj.name]);
            recovery_answers.erase(obj.name);
          }
          else
          {
            lost_ids.push_back(obj.name);
          }
        }
        else //TRACKER_TRACKING_MODE or TRACKER_LOCKED_MODE
        {
          should_process = true;
        }
      }
    }
    if(!lost_ids.empty())
    {
      boost::mutex::scoped_lock lock(recovery_mutex, boost::try_to_lock);
      if(lock)
      {
        // the recovery() function will be called with the result of "getRecoveryCall()" on a separate thread
        recovery_th = boost::thread(boost::bind(&TrackerNode::recovery, this, mode->getRecoveryCall(lost_ids, detectorImgMsg)));
      }
    }
    if(should_process)
    {
      ROS_INFO("----------------------------------------------");
      ROS_INFO("TrackerNode::imageCb: calling tracker->process");
      tracker->process(cv_tracker_ptr->image);
      BOOST_FOREACH(const blort::ObjectEntry& obj, tracker->getObjects())
      {
        if(tracker->getMode(obj.name) != blort_ros::TRACKER_RECOVERY_MODE && obj.is_tracked)
        {
          //confidences_pub.publish(*(tracker->getConfidences()[item.first]));
          if(tracker->getConfidence(obj.name) == blort_ros::TRACKER_CONF_GOOD ||
             (tracker->getConfidence(obj.name) == blort_ros::TRACKER_CONF_FAIR &&
              tracker->getPublishMode() == blort_ros::TRACKER_PUBLISH_GOOD_AND_FAIR) )
          {
            blort_msgs::TrackerResults msg;
            msg.obj_name.data = obj.name;
            msg.pose.header.seq = pose_seq++;
            msg.pose.header.stamp = ros::Time::now();
            msg.pose.header.frame_id = camera_frame_id;
            msg.pose.pose = blort_ros::blortPosesToRosPose(tracker->getCameraReferencePose(),
                                                           tracker->getDetections()[obj.name]);
            detection_result.publish(msg);
          }
        }
        cv_bridge::CvImage out_msg;
        out_msg.header = trackerImgMsg->header;
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = tracker->getImage();
        image_pub.publish(out_msg.toImageMsg());
      }
    }
  }
}

void TrackerNode::recovery(blort_msgs::RecoveryCall srv)
{
  boost::mutex::scoped_lock lock(recovery_mutex);
  ros::Time before = ros::Time::now();
  {
    std::stringstream ss;
    ss << "tracker_node calling detector_node recovery service for object(s): ";
    for(size_t i = 0; i < srv.request.object_ids.size();++i)
    {
      ss << srv.request.object_ids[i]  << ", ";
    }
    ROS_WARN_STREAM(ss.str());
  }
  if(recovery_client.call(srv))
  {
    for(size_t i = 0; i < srv.response.Poses.size(); ++i)
    {
      if(srv.response.object_founds[i])
      {
        recovery_answers[srv.response.object_ids[i]] = srv.response.Poses[i];
      }
    }
  }
  else
  {
    ROS_WARN("Detector not confident enough.");
  }
  ROS_INFO_STREAM("Recovery call took " << (ros::Time::now() - before));
}

bool TrackerNode::trackerControlServiceCb(blort_msgs::TrackerCommand::Request &req,
                                          blort_msgs::TrackerCommand::Response &)
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
TrackerNode::TrackingMode::TrackingMode(TrackerNode* parent) : parent_(parent)
{
  ROS_INFO("Blort tracker launched in tracking mode.");
  cam_info_sub = parent_->nh_.subscribe("/detector_camera_info", 10,
                                        &TrackerNode::TrackingMode::cam_info_callback, this);
}

void TrackerNode::TrackingMode::reconf_callback(blort_ros::TrackerConfig &config, uint32_t level)
{
  if(parent_->tracker != 0)
  {
    parent_->tracker->reconfigure(config);
  } else {
    ROS_WARN("Please publish camera_info for the tracker initialization.");
  }
}

blort_msgs::RecoveryCall TrackerNode::TrackingMode::getRecoveryCall(std::vector<std::string> & ids,
                                                                    const sensor_msgs::ImageConstPtr& msg)
{
  blort_msgs::RecoveryCall srv;
  srv.request.object_ids.resize(ids.size());
  for(size_t i = 0; i < ids.size(); ++i)
  {
    srv.request.object_ids[i] = ids[i];
  }
  srv.request.Image = *msg;
  return srv;
}

// The real initialization is being done after receiving the camerainfo.
void TrackerNode::TrackingMode::cam_info_callback(const sensor_msgs::CameraInfo &msg)
{
  if(parent_->tracker == 0)
  {
    ROS_INFO("Camera parameters received, ready to run.");
    parent_->setCameraFrameID(msg.header.frame_id);
    //parent_->_msg = msg; //2012-11-27 Jordi: keep a copy to reset the tracker when necessary
    cam_info_sub.shutdown();
    parent_->tracker = new blort_ros::GLTracker(msg, parent_->root_, true);
    parent_->tracker->setVisualizeObjPose(true);
    parent_->tracker->enableAllTracking(true);

    image_transport::TransportHints transportHint("raw");

    detector_image_sub.reset(new image_transport::SubscriberFilter(
                               parent_->it_, "/detector_image", 1, transportHint));
    tracker_image_sub.reset(new image_transport::SubscriberFilter(
                              parent_->it_, "/tracker_image", 1, transportHint));

    imageSynchronizer.reset( new message_filters::TimeSynchronizer<sensor_msgs::Image,
                             sensor_msgs::Image>(*detector_image_sub, *tracker_image_sub, 1) );

    imageSynchronizer->registerCallback(boost::bind(&TrackerNode::imageCb, parent_, _1, _2));

    parent_->control_service = parent_->nh_.advertiseService(
          "tracker_control", &TrackerNode::trackerControlServiceCb, parent_);
    parent_->server_ = std::auto_ptr<dynamic_reconfigure::Server<blort_ros::TrackerConfig> >
        (new dynamic_reconfigure::Server<blort_ros::TrackerConfig>());
    parent_->f_ = boost::bind(&TrackerNode::TrackingMode::reconf_callback, this, _1, _2);
    parent_->server_->setCallback(parent_->f_);
    parent_->recovery_client = parent_->nh_.serviceClient<blort_msgs::RecoveryCall>("/blort_detector/pose_service");
  }
}

TrackerNode::SingleShotMode::SingleShotMode(TrackerNode* parent)
  : as_(parent->nh_, "recognize_object", false)
  , parent_(parent)
{
  ROS_INFO("Blort tracker launched in singleshot mode.");
  detector_set_caminfo_service = parent_->nh_.serviceClient<blort_msgs::SetCameraInfo>("/blort_detector/set_camera_info");
  singleshot_service = parent_->nh_.advertiseService("singleshot_service", &TrackerNode::SingleShotMode::singleShotService, this);

  parent_->control_service = parent_->nh_.advertiseService("tracker_control", &TrackerNode::trackerControlServiceCb, parent_);
  parent_->server_ = std::auto_ptr<dynamic_reconfigure::Server<blort_ros::TrackerConfig> >
      (new dynamic_reconfigure::Server<blort_ros::TrackerConfig>());
  parent_->f_ = boost::bind(&SingleShotMode::reconf_callback, this, _1, _2);
  parent_->server_->setCallback(parent_->f_);

  time_to_run_singleshot = 10.;
  inServiceCall = false;
  parent->nh_.param<double>("conf_threshold", conf_treshold_, 0.3);

  as_.registerGoalCallback(boost::bind(&TrackerNode::SingleShotMode::goalCb, this, _1));
  as_.start();
}

void TrackerNode::SingleShotMode::reconf_callback(blort_ros::TrackerConfig &config, uint32_t level)
{
  time_to_run_singleshot =  config.time_to_run_singleshot;
}

blort_msgs::RecoveryCall TrackerNode::SingleShotMode::getRecoveryCall(std::vector<std::string> & ids,
                                                                      const sensor_msgs::ImageConstPtr& msg)
{
  blort_msgs::RecoveryCall srv;
  if(!inServiceCall)
  {
    srv.request.object_ids.resize(ids.size());
    for(size_t i = 0; i < ids.size(); ++i)
    {
      ROS_WARN_STREAM("TrackerNode::SingleShotMode::getRecoveryCall: "  << ids[i]);
      srv.request.object_ids[i] = ids[i];
    }
    srv.request.Image = *msg;
    // we step into the service call "state" with the first recoverycall made
    // no new images are accepted until the end of the singleShotServiceCall
    inServiceCall = true;
  }
  return srv;
}

/* FIXME Implement single-shot with object selection */
/* For now, single-shot runs on first object for backward compatibility */
bool TrackerNode::SingleShotMode::singleShotService(blort_msgs::EstimatePose::Request &req,
                                                    blort_msgs::EstimatePose::Response &resp)
{
  lastImage.reset();
  lastImage = ros::topic::waitForMessage<sensor_msgs::Image>("/detector_image", parent_->nh_, ros::Duration(1.0));
  lastCameraInfo.reset();
  lastCameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/detector_camera_info", parent_->nh_, ros::Duration(1.0));

  if(lastImage.use_count() < 1 && lastCameraInfo.use_count() < 1)
  {
    ROS_ERROR("Service called but there was no data on the input topics!");
    return false;
  }
  else
  {
    ROS_INFO("Singleshot service has been called with a timeout of %f seconds.", time_to_run_singleshot);
    results_list.clear();

    if(parent_->tracker == 0)
    {
      parent_->tracker = new blort_ros::GLTracker(*lastCameraInfo, parent_->root_, true);
      parent_->recovery_client = parent_->nh_.serviceClient<blort_msgs::RecoveryCall>("/blort_detector/pose_service");
    }
    else
    {
      parent_->tracker->reset();
      parent_->recovery_answers.clear();
    }
    //HACK: hardcoded tracking for pringles
    parent_->tracker->enableAllTracking(false);
    parent_->tracker->setTracked("Pringles", true);

    parent_->tracker->setPublishMode(blort_ros::TRACKER_PUBLISH_GOOD);
    parent_->tracker->setVisualizeObjPose(true);
    blort_msgs::SetCameraInfo camera_info;
    camera_info.request.CameraInfo = *lastCameraInfo;
    if(!detector_set_caminfo_service.call(camera_info))
      ROS_ERROR("blort_tracker failed to call blort_detector/set_camera_info service");

    double start_secs = ros::Time::now().toSec();
    while(ros::Time::now().toSec()-start_secs < time_to_run_singleshot)
    {
      ROS_INFO("Remaining time %f", time_to_run_singleshot+start_secs-ros::Time::now().toSec());
      parent_->imageCb(lastImage, lastImage);
      if(parent_->tracker->getConfidence("Pringles") == blort_ros::TRACKER_CONF_FAIR) // HACK
      {
        // instead of returning right away let's store the result
        // to see if the tracker can get better
        results_list.push_back(parent_->tracker->getDetections()["Pringles"]); // HACK
      }
      else if(parent_->tracker->getConfidence("Pringles") == blort_ros::TRACKER_CONF_LOST) // HACK
      {
        results_list.clear();
      }
    }
    // we are out of the service call now, the results will be published
    inServiceCall = false;
    if(!results_list.empty())
    {
      //convert results to a tf style transform and multiply them
      //to get the camera-to-target transformation
      resp.Pose = blort_ros::blortPosesToRosPose(parent_->tracker->getCameraReferencePose(),
                                                 results_list.back());
      //NOTE: check the pose in vec3 location + mat3x3 rotation could be added here
      // if we have any previous knowledge of the given scene
      ROS_INFO_STREAM("PUBLISHED POSE:" << std::endl << resp.Pose.position << std::endl <<
                      blort_ros::quaternionTo3x3cvMat(resp.Pose.orientation) << std::endl);
      return true;
    }
    else
    {
      //if the time was not enough to get a good detection, make the whole thing fail
      return false;
    }
  }
}

void TrackerNode::SingleShotMode::goalCb(AcServer::GoalHandle gh)
{
  AcServer::GoalConstPtr goal = gh.getGoal();
  gh.setAccepted();
  result_.recognized_objects.objects.clear();

  if(goal->objects.empty())
  {
    gh.setSucceeded(result_);
    return;
  }

  lastImage.reset();
  lastImage = ros::topic::waitForMessage<sensor_msgs::Image>("/detector_image", parent_->nh_, ros::Duration(3.0));
  sensor_msgs::ImageConstPtr lastDepth = ros::topic::waitForMessage<sensor_msgs::Image>("/depth_image", parent_->nh_, ros::Duration(3.0));
  lastCameraInfo.reset();
  lastCameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/detector_camera_info", parent_->nh_, ros::Duration(3.0));
  if(lastImage.use_count() < 1 && lastCameraInfo.use_count() < 1)
  {
    ROS_ERROR("Action called but there was no data on the input topics!");
    gh.setAborted();
    return;
  }

  if(goal->refine_pose_time <= 0)
  {
    ROS_ERROR("Refine pose time must be positive.");
    gh.setRejected();
    return;
  }


  // RUN blort tracking mechanism
  // initialize tracker if it wasn't, otherwise reset it
  if(parent_->tracker != 0)
    delete parent_->tracker;

  parent_->tracker = new blort_ros::GLTracker(*lastCameraInfo, parent_->root_, true);
  parent_->recovery_client = parent_->nh_.serviceClient<blort_msgs::RecoveryCall>("/blort_detector/pose_service");
  parent_->recovery_answers.clear();
  parent_->tracker->enableAllTracking(false);

  // validate goal
  BOOST_FOREACH(const object_recognition_msgs::ObjectType& obj_type, goal->objects)
  {
    bool found = false;
    BOOST_FOREACH(const blort::ObjectEntry& obj, parent_->tracker->getObjects())
    {
      if(obj.name == obj_type.key)
      {
        found = true;
        break;
      }
    }
    if(!found)
    {
      ROS_ERROR_STREAM("Unknown object called: " << obj_type.key);
      gh.setAborted();
      return;
    }
  }

  std::stringstream ss;
  BOOST_FOREACH(const object_recognition_msgs::ObjectType& obj_type, goal->objects)
  {
    parent_->tracker->setTracked(obj_type.key, true);
    ss << obj_type.key << ",";
  }
  ROS_INFO_STREAM("TrackerNode::SingleShotMode::goalCb(): Received request to recognize '"
                  << ss.str() <<"' in " << goal->refine_pose_time << "seconds.");


  parent_->tracker->setPublishMode(blort_ros::TRACKER_PUBLISH_GOOD_AND_FAIR);
  parent_->tracker->setVisualizeObjPose(true);
  blort_msgs::SetCameraInfo camera_info;
  camera_info.request.CameraInfo = *lastCameraInfo;
  if(!detector_set_caminfo_service.call(camera_info))
    ROS_ERROR("blort_tracker failed to call blort_detector/set_camera_info service");

  std::map< std::string, std::vector<geometry_msgs::Pose> > results;
  double start_secs = ros::Time::now().toSec();
  while(ros::Time::now().toSec()-start_secs < goal->refine_pose_time)
  {
    // feedback
    feedback_.time_left = goal->refine_pose_time+start_secs-ros::Time::now().toSec();
    gh.publishFeedback(feedback_);
    ROS_INFO("Remaining time %f", feedback_.time_left);

    parent_->imageCb(lastImage, lastImage);

    BOOST_FOREACH(const blort::ObjectEntry& obj, parent_->tracker->getObjects())
    {
      bool found = false;
      BOOST_FOREACH(const object_recognition_msgs::ObjectType& obj_type, goal->objects)
      {
        found = found || (obj.name == obj_type.key);
      }
      if(found)
      {
        ROS_ERROR_STREAM(obj.name << " with confidence " << obj.edgeConf
                         << ", when threshold is " << conf_treshold_);
        // Check that the edge confidence is high enought BUT ALSO that the pose
        // has been updated. If not, it means that model confidence state is not
        // the required by the /blort_tracker/publish_mode
        if(obj.edgeConf > conf_treshold_ &&
           !(parent_->tracker->getDetections()[obj.name].position.x == 0 &&
             parent_->tracker->getDetections()[obj.name].position.y == 0 &&
             parent_->tracker->getDetections()[obj.name].position.z == 0) )
        {
          // instead of returning right away let's store the result
          // to see if the tracker can get better
          results[obj.name].push_back(parent_->tracker->getDetections()[obj.name]);
        }
        else
        {
          results[obj.name].clear();
        }
      }
    }
  }
  inServiceCall = false;

  // time is up, pose refinement is done
  BOOST_FOREACH(const object_recognition_msgs::ObjectType& obj, goal->objects)
  {
    if(!results[obj.key].empty())
    {
      object_recognition_msgs::RecognizedObject r_obj;
      //pose.header.frame_id = camera frame name
      r_obj.pose.header.frame_id = lastCameraInfo->header.frame_id;
      r_obj.pose.header.stamp = ros::Time::now(); // or take the stamp of the last image
      r_obj.type.key = obj.key;
      //convert results to a tf style transform and multiply them
      //to get the camera-to-target transformation
      r_obj.pose.pose.pose = blort_ros::blortPosesToRosPose(parent_->tracker->getCameraReferencePose(),
                                                            results[obj.key].back());
      //copy Z element of depth[X,Y]
      if(lastDepth.use_count() > 0)
      {
        r_obj.pose.pose.pose.position.z = getDistance(lastDepth,
                                                      r_obj.pose.pose.pose.position.x,
                                                      r_obj.pose.pose.pose.position.y,
                                                      r_obj.pose.pose.pose.position.z);
      }
      result_.recognized_objects.objects.push_back(r_obj);
      //NOTE: check the pose in vec3 location + mat3x3 rotation could be added here
      // if we have any previous knowledge of the given scene
      //            ROS_INFO_STREAM("PUBLISHED POSE:" << std::endl << resp.Pose.position << std::endl <<
      //                            pal_blort::quaternionTo3x3cvMat(resp.Pose.orientation) << std::endl);
    }
  }
  gh.setSucceeded(result_);
}

double TrackerNode::SingleShotMode::getDistance(const sensor_msgs::ImageConstPtr& img, double x, double y, double z)
{
  double u = (lastCameraInfo->P[0]*x)/z + lastCameraInfo->P[2];
  double v = (lastCameraInfo->P[5]*y)/z + lastCameraInfo->P[6];

  cv_bridge::CvImagePtr depth_ptr;
  try
  {
    depth_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
    ROS_WARN_STREAM("[x,y,z] : " << "[" << x << "," << y << "," << z << "]   --- " <<
                    "[u,v] : [" << u << "," << v << "] in an image of [" << depth_ptr->image.rows << "," << depth_ptr->image.cols << "]");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return 0.0;
  }

  // compute the median of the 3x3 neighborhood of the point
  float arr[] = {depth_ptr->image.at<float>(round(v)-1, round(u)-1), depth_ptr->image.at<float>(round(v)-1, round(u)),
                 depth_ptr->image.at<float>(round(v)-1, round(u)+1), depth_ptr->image.at<float>(round(v), round(u)-1),
                 depth_ptr->image.at<float>(round(v), round(u)), depth_ptr->image.at<float>(round(v), round(u)+1),
                 depth_ptr->image.at<float>(round(v)+1, round(u)-1), depth_ptr->image.at<float>(round(v)+1, round(u)),
                 depth_ptr->image.at<float>(round(v)+1, round(u)+1)};
  std::nth_element(arr, arr+5, arr+9);

  // print neighbouring elements
  //  ROS_ERROR_STREAM(
  //        "-| " << depth_ptr->image.at<float>(round(v)-1, round(u)-1) << " | " << depth_ptr->image.at<float>(round(v)-1, round(u)) <<  " | " << depth_ptr->image.at<float>(round(v)-1, round(u)+1) <<  " |-" <<
  //        "-| " << depth_ptr->image.at<float>(round(v), round(u)-1) << " | " << depth_ptr->image.at<float>(round(v), round(u)) <<  " | " << depth_ptr->image.at<float>(round(v), round(u)+1) <<  " |-" <<
  //        "-| " << depth_ptr->image.at<float>(round(v)+1, round(u)-1) << " | " << depth_ptr->image.at<float>(round(v)+1, round(u)) <<  " | " << depth_ptr->image.at<float>(round(v)+1, round(u)+1) <<  " |-"
  //                  );
  ROS_WARN_STREAM("Median is " << arr[5]);

  return arr[5];
}


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
  blort_ros::GLXHidingWindow window(656, 492, "Tracker"); // a window which should hide itself after start
  //blortGLWindow::GLWindow window(640  , 480, "Window"); // a normal opengl window
  TrackerNode node(argv[1]);
  ros::spin();
  return 0;
}

