#include "tracker_node.h"

TrackerNode::TrackerNode(std::string root)
    : nh_("blort_tracker"), it_(nh_), pose_seq(0), camera_frame_id("0"), root_(root), tracker(0)
{
    nh_.param<std::string>("launch_mode", launch_mode, "tracking");
    detection_result = nh_.advertise<blort_ros::TrackerResults>("detection_result", 100);
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

void TrackerNode::imageCb(const sensor_msgs::ImageConstPtr& detectorImgMsg, const sensor_msgs::ImageConstPtr& trackerImgMsg )
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
        for(size_t i = 0; i < tracker->getModes().size(); ++i)
        {
            if(tracker->getModes()[i] == blort_ros::TRACKER_RECOVERY_MODE)
            {              
                blort_ros::RecoveryCall srv = mode->recovery(i, detectorImgMsg);

                ros::Time before = ros::Time::now();
                ROS_INFO_STREAM("tracker_node in TRACKER_RECOVERY_MODE: calling detector_node recovery service for object " << tracker->getModelNames()[i]);
                if(recovery_client.call(srv))
                {
                  //tracker = new blort_ros::GLTracker(_msg, root_, true);
                  ROS_INFO("reseting tracker with pose from detector");
                  tracker->resetWithPose(i, srv.response.Pose);
                  ROS_INFO("AFTER reseting tracker with pose from detector");
                }
                else
                {
                    ROS_WARN("Detector not confident enough.");
                }
                ROS_INFO_STREAM("Recovery call took " << (ros::Time::now() - before));
            }
            else //TRACKER_TRACKING_MODE or TRACKER_LOCKED_MODE
            {
              should_process = true;
            }
        }
        if(should_process)
        {
            ROS_INFO("----------------------------------------------");
            ROS_INFO("TrackerNode::imageCb: calling tracker->process");
            tracker->process(cv_tracker_ptr->image);
            for(size_t i = 0; i < tracker->getModes().size(); ++i)
            {
                  if(tracker->getModes()[i] == blort_ros::TRACKER_RECOVERY_MODE)
                  {
                      continue;
                  }
                  confidences_pub.publish(*(tracker->getConfidences()[i]));
                  if(tracker->getConfidence()[i] == blort_ros::TRACKER_CONF_GOOD ||
                      (tracker->getConfidence()[i] == blort_ros::TRACKER_CONF_FAIR && tracker->getPublishMode() == blort_ros::TRACKER_PUBLISH_ALL) )
                  {
                      blort_ros::TrackerResults msg;
                      msg.obj_name.data = tracker->getModelNames()[i];
                      msg.pose.header.seq = pose_seq++;
                      msg.pose.header.stamp = ros::Time::now();
                      msg.pose.header.frame_id = camera_frame_id;
                      msg.pose.pose = pal_blort::blortPosesToRosPose(tracker->getCameraReferencePose(),
                                                                                       tracker->getDetections()[i]);

                      detection_result.publish(msg);
                  }
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

bool TrackerNode::trackerControlServiceCb(blort_ros::TrackerCommand::Request &req,
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
TrackerNode::TrackingMode::TrackingMode(TrackerNode* parent) : parent_(parent)
{
    ROS_INFO("Blort tracker launched in tracking mode.");
    cam_info_sub = parent_->nh_.subscribe("/detector_camera_info", 10, &TrackerNode::TrackingMode::cam_info_callback, this);
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

blort_ros::RecoveryCall TrackerNode::TrackingMode::recovery(size_t i, const sensor_msgs::ImageConstPtr& msg)
{
    blort_ros::RecoveryCall srv;
    srv.request.object_id.data = i;
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

TrackerNode::SingleShotMode::SingleShotMode(TrackerNode* parent) : parent_(parent)
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

void TrackerNode::SingleShotMode::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    if(!inServiceCall)
    {
        // if there is no ongoing servicecall, we can receive the new infos
        // if we are in an ongoing servicecall, these fields should be constant
        lastImage = image;
    }
}

void TrackerNode::SingleShotMode::cameraCallback(const sensor_msgs::CameraInfoConstPtr &camera_info)
{
    if(!inServiceCall)
    {
        // if there is no ongoing servicecall, we can receive the new infos
        // if we are in an ongoing servicecall, these fields should be constant
        lastCameraInfo = camera_info;
    }
}

void TrackerNode::SingleShotMode::reconf_callback(blort_ros::TrackerConfig &config, uint32_t level)
{
    time_to_run_singleshot =  config.time_to_run_singleshot;
}

blort_ros::RecoveryCall TrackerNode::SingleShotMode::recovery(size_t i, const sensor_msgs::ImageConstPtr& msg)
{
    blort_ros::RecoveryCall srv;
    if(!inServiceCall)
    {
        srv.request.object_id.data = i;
        srv.request.Image = *msg;
        // we step into the service call "state" with the first recoverycall made
        // no new images are accepted until the end of the singleShotServiceCall
        inServiceCall = true;
    }
    return srv;
}

/* FIXME Implement single-shot with object selection */
/* For now, single-shot runs on first object for backward compatibility */
bool TrackerNode::SingleShotMode::singleShotService(blort_ros::EstimatePose::Request &req,
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
            if(parent_->tracker->getConfidence()[0] == blort_ros::TRACKER_CONF_GOOD)
            {
                // instead of returning right away let's store the result
                // to see if the tracker can get better
                results.push_back(parent_->tracker->getDetections()[0]);
            } else if(parent_->tracker->getConfidence()[0] == blort_ros::TRACKER_CONF_LOST)
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
            ROS_INFO_STREAM("PUBLISHED POSE:" << std::endl << resp.Pose.position << std::endl <<
                            pal_blort::quaternionTo3x3cvMat(resp.Pose.orientation) << std::endl);
            return true;
        } else {
            //if the time was not enough to get a good detection, make the whole thing fail
            return false;
        }
    }
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
    pal_blort::GLXHidingWindow window(656, 492, "Tracker"); // a window which should hide itself after start
    //blortGLWindow::GLWindow window(640  , 480, "Window"); // a normal opengl window
    TrackerNode node(argv[1]);
    ros::spin();
    return 0;
}

