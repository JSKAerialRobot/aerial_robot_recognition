#include <aerial_robot_perception/depth_distance_filter.h>

namespace aerial_robot_perception
{

  void DepthDistanceFilter::onInit()
  {
    DiagnosticNodelet::onInit();

    pnh_->param("min_depth", min_depth_, 0.01);
    pnh_->param("max_depth", max_depth_, 1.0);
    pnh_->param("distance_from_ground", distance_from_ground_, 0.02);
    pnh_->param("use_distance_from_ground", use_distance_from_ground_, true);
    pnh_->param("use_otsu_binarization", use_otsu_binarization_, false);


    tf_ls_ = std::make_shared<tf2_ros::TransformListener>(tf_buff_);

    it_ = std::make_shared<image_transport::ImageTransport>(*pnh_);

    dr_server_ = std::make_shared<dynamic_reconfigure::Server<aerial_robot_perception::DepthDistanceFilterConfig> >(*pnh_);
    dynamic_reconfigure::Server<aerial_robot_perception::DepthDistanceFilterConfig>::CallbackType f = boost::bind(&DepthDistanceFilter::reconfigureCallback, this, _1, _2);
    dr_server_->setCallback(f);

    image_pub_ = advertiseImage(*pnh_, "output", 1);

    ros::Duration(1.0).sleep();

    onInitPostProcess();
  }

  void DepthDistanceFilter::subscribe()
  {
    image_sub_ = it_->subscribe("input", 1, &DepthDistanceFilter::depthImageCallback, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void DepthDistanceFilter::unsubscribe()
  {
    image_sub_.shutdown();
  }

  void DepthDistanceFilter::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    double max_depth_local = max_depth_;
    if (use_distance_from_ground_) {
      tf2::Transform cam_tf;
      try {
        geometry_msgs::TransformStamped cam_pose_msg = tf_buff_.lookupTransform("world", msg->header.frame_id, ros::Time(0), ros::Duration(0.1));
        tf2::convert(cam_pose_msg.transform, cam_tf);

        max_depth_local = cam_tf.getOrigin().z() - distance_from_ground_;
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
      }
    }

    cv::Mat depth_img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    if (msg->encoding == "8UC1" || msg->encoding == "16UC1" || msg->encoding == "32UC1") {
      depth_img.convertTo(depth_img, CV_32FC1, 0.001);
    }

    cv::Mat output_img;

    if (use_otsu_binarization_) {
      double max_val;
      cv::minMaxLoc(depth_img, NULL, &max_val);
      cv::Mat bin_img;
      cv::threshold(depth_img, bin_img, min_depth_, max_val, cv::THRESH_BINARY_INV);
      cv::threshold(depth_img, depth_img, min_depth_, max_val, cv::THRESH_TOZERO);
      depth_img += bin_img;
      cv::normalize(depth_img, depth_img, 0, 255, cv::NORM_MINMAX);
      depth_img.convertTo(depth_img, CV_8UC1);

      cv::threshold(depth_img, output_img, 0, 255, cv::THRESH_BINARY_INV|cv::THRESH_OTSU);
    } else {
      cv::threshold(depth_img, depth_img, min_depth_, 0, cv::THRESH_TOZERO);
      cv::threshold(depth_img, depth_img, max_depth_local, 0, cv::THRESH_TOZERO_INV);
      cv::threshold(depth_img, depth_img, 0, 255, cv::THRESH_BINARY);
      depth_img.convertTo(output_img, CV_8UC1);
    }

    sensor_msgs::ImagePtr output_img_msg = cv_bridge::CvImage(msg->header, "mono8", output_img).toImageMsg();
    image_pub_.publish(output_img_msg);
  }

  void DepthDistanceFilter::reconfigureCallback(aerial_robot_perception::DepthDistanceFilterConfig &config, uint32_t level)
  {
    min_depth_ = config.min_depth;
    max_depth_ = config.max_depth;
    distance_from_ground_ = config.distance_from_ground;
    use_distance_from_ground_ = config.use_distance_from_ground;
    use_otsu_binarization_ = config.use_otsu_binarization;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aerial_robot_perception::DepthDistanceFilter, nodelet::Nodelet);
