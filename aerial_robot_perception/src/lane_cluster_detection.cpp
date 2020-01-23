#include <aerial_robot_perception/lane_cluster_detection.h>

namespace aerial_robot_perception
{

  void LaneClusterDetection::onInit()
  {
    DiagnosticNodelet::onInit();
    
    pnh_->param("debug_view", debug_view_, true);
    pnh_->param("frame_id", frame_id_, std::string("target_object"));

    pnh_->param("wall_height", wall_height_, 1.7);
    pnh_->param("always_subscribe", always_subscribe_, true);
    if (debug_view_) debug_image_pub_ = advertiseImage(*pnh_, "debug_image", 1);
    target_pub_ = advertise<geometry_msgs::Vector3Stamped>(*pnh_, frame_id_, 1);

    it_ = boost::make_shared<image_transport::ImageTransport>(*pnh_);
    tf_ls_ = boost::make_shared<tf2_ros::TransformListener>(tf_buff_);

    ros::Duration(1.0).sleep();

    onInitPostProcess();
  }

  void LaneClusterDetection::subscribe()
  {
    if(debug_view_) rgb_image_sub_ = it_->subscribe("rgb_img", 1, &LaneClusterDetection::rgbImageCallback, this);
    cam_info_sub_ = nh_->subscribe("cam_info", 1, &LaneClusterDetection::cameraInfoCallback, this);
    mask_image_sub_ = it_->subscribe("input", 1, &LaneClusterDetection::maskImageCallback, this);
  }

  void LaneClusterDetection::unsubscribe()
  {
    rgb_image_sub_.shutdown();
    mask_image_sub_.shutdown();
  }

  void LaneClusterDetection::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    rgb_img_ = cv_ptr->image;
  }
  
  void LaneClusterDetection::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    /* the following process is executed once */
    NODELET_DEBUG_STREAM("receive camera info");
    tf2::Matrix3x3 camera_K_normal(msg->K[0], msg->K[1], msg->K[2],
                            msg->K[3], msg->K[4], msg->K[5],
                            msg->K[6], msg->K[7], msg->K[8]);

    camera_K_inv_ = camera_K_normal.inverse();
    camera_K = camera_K_normal;
    real_size_scale_ = msg->K[0] * msg->K[4];
    camera_optical_frame_name_ = msg->header.frame_id;
    cam_info_sub_.shutdown();
  }

  void LaneClusterDetection::maskImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(real_size_scale_ == 0) {
      NODELET_DEBUG_STREAM("real_size_scale is 0");
      return;
    }

    tf2::Transform cam_tf;
    try {
      geometry_msgs::TransformStamped cam_pose_msg = tf_buff_.lookupTransform("world", camera_optical_frame_name_, ros::Time(0), ros::Duration(0.1));
      tf2::convert(cam_pose_msg.transform, cam_tf);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }

    cv::Mat mask_img = cv_bridge::toCvCopy(msg, msg->encoding)->image;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  //  find all contours

    if (contours.size() == 0) {
      NODELET_DEBUG_STREAM("no contours");
      return;  //  no contours -> return
    }

    double object_distance = cam_tf.getOrigin().z() - wall_height_;
    cv::Moments contour_moments = cv::moments(contours[0], true);
    tf2::Vector3 target_obj_uv;
    target_obj_uv.setX(contour_moments.m10 / contour_moments.m00);
    target_obj_uv.setY(contour_moments.m01 / contour_moments.m00);
    target_obj_uv.setZ(1.0);

    tf2::Vector3 object_pos_in_optical_frame = camera_K_inv_ * target_obj_uv * object_distance;

    geometry_msgs::TransformStamped obj_tf_msg;
    obj_tf_msg.header = msg->header;
    obj_tf_msg.header.frame_id = camera_optical_frame_name_;
    obj_tf_msg.child_frame_id = frame_id_;
    obj_tf_msg.transform.translation = tf2::toMsg(object_pos_in_optical_frame);
    obj_tf_msg.transform.rotation.w = 1.0;
    tf_br_.sendTransform(obj_tf_msg);

    geometry_msgs::Vector3Stamped obj_pos_msg;
    obj_pos_msg.header = obj_tf_msg.header;
    obj_pos_msg.vector = obj_tf_msg.transform.translation;
    target_pub_.publish(obj_pos_msg);

    if(debug_view_){
      cv::Mat debug_img;
      rgb_img_.copyTo(debug_img);
      cv::drawContours(debug_img, contours, -1, cv::Scalar(255,0,0));
      cv::circle(debug_img, cv::Point(contour_moments.m10 / contour_moments.m00, contour_moments.m01 / contour_moments.m00), 20, cv::Scalar(0, 0, 255), 20);
      sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_img).toImageMsg();
      debug_image_pub_.publish(debug_img_msg);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aerial_robot_perception::LaneClusterDetection, nodelet::Nodelet);
