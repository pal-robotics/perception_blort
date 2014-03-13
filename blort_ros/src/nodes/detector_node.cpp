#include <blort_ros/detector_node.h>

DetectorNode::DetectorNode(std::string root)
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

DetectorNode::~DetectorNode()
{
    if(detector != 0)
        delete(detector);
}

bool DetectorNode::recovery(blort_ros_msgs::RecoveryCall::Request &req,
              blort_ros_msgs::RecoveryCall::Response &resp)
{
    if(detector != 0)
    {
        ROS_INFO("Detector called the %u-th time.", ++counter);
        bool result;
        resp.object_ids = req.object_ids;
        std::vector<size_t> object_ids;
        for(size_t i = 0; i < req.object_ids.size(); ++i)
        {
            object_ids.push_back(req.object_ids[i].data);
        }
        if(!req.Image.data.empty())
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
              cv_ptr = cv_bridge::toCvCopy(req.Image, sensor_msgs::image_encodings::BGR8);
              ROS_INFO_STREAM("\n\nENCODING = " << req.Image.encoding << "\n\n\n");
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return false;
            }
            result = detector->recovery(object_ids, cv_ptr->image, resp);
        } else {
            ROS_INFO("Running detector on latest image.");
            result = detector->recoveryWithLast(object_ids, resp);
        }
        cv_bridge::CvImage out_msg;
        out_msg.header = req.Image.header;
        out_msg.header.stamp = ros::Time::now();
        //out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = detector->getImage();
        image_pub.publish(out_msg.toImageMsg());
        ROS_INFO_STREAM("= > detector_node returned " << result);

        return result;
    } else {
        ROS_ERROR("blort_detector service called while the core is uninitialized.");
        return false;
    }
}

bool DetectorNode::setCameraInfoCb(blort_ros_msgs::SetCameraInfo::Request &req,
                     blort_ros_msgs::SetCameraInfo::Response &resp)
{
    if(detector == 0)
        cam_info_callback(req.CameraInfo);
    return true;
}

void DetectorNode::reconf_callback(blort_ros::DetectorConfig &config, uint32_t level)
{
    if(detector != 0)
    {
        detector->reconfigure(config);
        ROS_INFO("Detector confidence threshold set to %f", config.recovery_conf_threshold);
    } else {
        ROS_WARN("Please publish camera_info for the tracker initialization.");
    }
}

void DetectorNode::cam_info_callback(const sensor_msgs::CameraInfo &msg)
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

