#include <aerial_robot_perception/rectangle_detection.h>

namespace aerial_robot_perception
{

  void RectangleDetection::onInit()
  {
    DiagnosticNodelet::onInit();

    pnh_->param("debug_view", debug_view_, true);
    pnh_->param("frame_id", frame_id_, std::string("target"));
    pnh_->param("lowest_margin", lowest_margin_, 10);
    pnh_->param("object_height", object_height_, 0.20);
    pnh_->param("target_object_area", target_object_area_, 0.06);
    pnh_->param("target_object_area_margin", target_object_area_margin_, 0.02);

    if (debug_view_) debug_image_pub_ = advertiseImage(*pnh_, "debug_image", 1);
    target_pub_ = advertise<geometry_msgs::PoseArray>(*pnh_, frame_id_, 1);

    it_ = std::make_shared<image_transport::ImageTransport>(*pnh_);
    tf_ls_ = std::make_shared<tf2_ros::TransformListener>(tf_buff_);

    dr_server_ = std::make_shared<dynamic_reconfigure::Server<aerial_robot_perception::RectangleDetectionConfig> >(*pnh_);
    dynamic_reconfigure::Server<aerial_robot_perception::RectangleDetectionConfig>::CallbackType f = boost::bind(&RectangleDetection::reconfigureCallback, this, _1, _2);
    dr_server_->setCallback(f);

    ros::Duration(1.0).sleep();

    onInitPostProcess();
  }

  void RectangleDetection::subscribe()
  {
    if(debug_view_) rgb_image_sub_ = it_->subscribe("rgb_img", 1, &RectangleDetection::rgbImageCallback, this);
    mask_image_sub_ = it_->subscribe("input", 1, &RectangleDetection::maskImageCallback, this);
    cam_info_sub_ = pnh_->subscribe("cam_info", 1, &RectangleDetection::cameraInfoCallback, this);
  }

  void RectangleDetection::unsubscribe()
  {
    rgb_image_sub_.shutdown();
    mask_image_sub_.shutdown();
  }


  void RectangleDetection::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    /* the following process is executed once */
    NODELET_DEBUG_STREAM("receive camera info");
    tf2::Matrix3x3 camera_K_normal(msg->K[0], msg->K[1], msg->K[2],
                            msg->K[3], msg->K[4], msg->K[5],
                            msg->K[6], msg->K[7], msg->K[8]);

    camera_K_inv_ = camera_K_normal.inverse();
    camera_K = camera_K_normal;
    image_height_ = msg->height;
    image_width_ = msg->width;
    real_size_scale_ = msg->K[0] * msg->K[4];
    camera_optical_frame_name_ = msg->header.frame_id;
    cam_info_sub_.shutdown();
  }


  void RectangleDetection::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    rgb_img_ = cv_ptr->image;
  }


  void RectangleDetection::maskImageCallback(const sensor_msgs::ImageConstPtr& msg)
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

    double object_distance = cam_tf.getOrigin().z() - object_height_;
    std::vector<cv::RotatedRect> rects;
    for (int i = 0; i < contours.size(); i++) {
      cv::Mat input_points;
      cv::Mat(contours[i]).convertTo(input_points, CV_32F);
      cv::RotatedRect rect = cv::minAreaRect(input_points);  //  detect rectangles(with degree)

      cv::Point2f vertices[4];
      rect.points(vertices);  //  get rect vertices

      //  check whether target object or not based on its area
      std::vector<tf2::Vector3> rect_points;
      for (int j = 0; j < 3; j++) {
        tf2::Vector3 rect_point_img;
        rect_point_img.setX(vertices[j].x);
        rect_point_img.setY(vertices[j].y);
        rect_point_img.setZ(1.0);

        tf2::Vector3 rect_point = camera_K_inv_ * rect_point_img * object_distance;
        rect_points.push_back(rect_point);
      }

      double rect_area = std::sqrt(std::pow((rect_points[0].x()-rect_points[1].x()), 2.0)+std::pow((rect_points[0].y()-rect_points[1].y()), 2.0)) * std::sqrt(std::pow((rect_points[1].x()-rect_points[2].x()), 2.0)+std::pow((rect_points[1].y()-rect_points[2].y()), 2.0));

      if (std::abs(rect_area - target_object_area_) > target_object_area_margin_) continue;
      rects.push_back(rect);
    }

    if (rects.size() == 0) {
      NODELET_DEBUG_STREAM("no valid size rects");
      return; // no rects -> return
    }

    //exclude too low lect in x & y axis

    std::vector<cv::RotatedRect> passed_rects;
    for (const auto& rect : rects) {
      cv::Point2f vertices[4];
      rect.points(vertices);  //  get rect vertices

      bool rect_ok = true;
      for (int i = 0; i < 4; ++i) {
        if (vertices[i].x < lowest_margin_ || (image_width_ - vertices[i].x) < lowest_margin_ || vertices[i].y < lowest_margin_ || (image_height_ - vertices[i].y) < lowest_margin_) {
          rect_ok = false;
        }
      }
      if (rect_ok) {
        passed_rects.push_back(rect);
      }
    }

    if (passed_rects.size() == 0) {
      NODELET_DEBUG_STREAM("no valid position rects");
      return; // no rects -> return
    }

    //publish poses of rectangles
    int obj_count = 0;
    geometry_msgs::PoseArray pose_array_msg;
    pose_array_msg.header = msg->header;
    pose_array_msg.header.frame_id = camera_optical_frame_name_;
    std::vector<double> angles;

    for (const auto& rect : passed_rects) {
      tf2::Vector3 target_obj_uv(rect.center.x, rect.center.y, 1.0);
      tf2::Vector3 object_pos_in_optical_frame = camera_K_inv_ * target_obj_uv * object_distance;

      //calc angle : x axis directs the long side
      cv::Point2f vertices[4];
      rect.points(vertices);
      double side1_len = cv::norm(vertices[0] - vertices[1]);
      double side2_len = cv::norm(vertices[1] - vertices[2]);
      cv::Point2f angle_vector;
      if (side1_len >= side2_len) {
        angle_vector = vertices[0] - vertices[1];
      } else {
        angle_vector = vertices[1] - vertices[2];
      }
      double angle = std::atan2(angle_vector.y, angle_vector.x);
      if (0.0 <= angle && angle < M_PI) {
        angle += M_PI;
      }

      angles.push_back(angle);

      std::ios::fmtflags curret_flag = std::cout.flags();
      std::ostringstream ss;
      ss << std::setw(2) << std::setfill('0') << obj_count;
      std::cout.flags(curret_flag);

      geometry_msgs::TransformStamped obj_tf_msg;
      obj_tf_msg.header = msg->header;
      obj_tf_msg.header.frame_id = camera_optical_frame_name_;
      obj_tf_msg.child_frame_id = frame_id_ + ss.str();
      obj_tf_msg.transform.translation = tf2::toMsg(object_pos_in_optical_frame);
      tf2::Quaternion q;
      q.setRPY(0, 0, angle);
      obj_tf_msg.transform.rotation = tf2::toMsg(q);
      tf_br_.sendTransform(obj_tf_msg);

      geometry_msgs::Pose pose;
      pose.position.x = obj_tf_msg.transform.translation.x;
      pose.position.y = obj_tf_msg.transform.translation.y;
      pose.position.z = obj_tf_msg.transform.translation.z;
      pose.orientation = obj_tf_msg.transform.rotation;
      pose_array_msg.poses.push_back(pose);
      obj_count++;
    }

    target_pub_.publish(pose_array_msg);

    //  draw all target rects(for debug)
    if (debug_view_) {
      cv::Mat debug_img;
      rgb_img_.copyTo(debug_img);
      for (int i = 0; i < passed_rects.size(); ++i) {
        cv::Point2f vertices[4];
        passed_rects[i].points(vertices);
        for (int i = 0; i < 4; ++i) {
          cv::line(debug_img, vertices[i], vertices[(i+1)%4], cv::Scalar(255,0,0), 10); //blue
        }
        cv::Point2f arrow_point = cv::Point2f(50.0 * std::cos(angles[i]), 50.0 * std::sin(angles[i])) + passed_rects[i].center;
        cv::arrowedLine(debug_img, passed_rects[i].center, arrow_point, cv::Scalar(0, 0, 0), 3, CV_AA, 0, 0.4);
      }
      sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_img).toImageMsg();
      debug_image_pub_.publish(debug_img_msg);
    }


  }

  void RectangleDetection::reconfigureCallback(aerial_robot_perception::RectangleDetectionConfig &config, uint32_t level)
  {
    lowest_margin_ = config.lowest_margin;
    object_height_ = config.object_height;
    target_object_area_ = config.target_object_area;
    target_object_area_margin_ = config.target_object_area_margin;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aerial_robot_perception::RectangleDetection, nodelet::Nodelet);
