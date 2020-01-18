#include <aerial_robot_perception/rectangle_detection_depth.h>

namespace aerial_robot_perception
{

  void RectangleDetectionDepth::onInit()
  {
    DiagnosticNodelet::onInit();

    pnh_->param("debug_view", debug_view_, true);
    pnh_->param("frame_id", frame_id_, std::string("target"));
    pnh_->param("lowest_margin", lowest_margin_, 10);
    pnh_->param("object_height", object_height_, 0.20);
    pnh_->param("target_object_area", target_object_area_, 0.06);
    pnh_->param("target_object_area_margin", target_object_area_margin_, 0.02);
    always_subscribe_ = true;

    if (debug_view_) image_pub_ = advertiseImage(*pnh_, "debug_image", 1);
    target_pub_ = advertise<geometry_msgs::PoseArray>(*pnh_, frame_id_, 1);

    it_ = boost::make_shared<image_transport::ImageTransport>(*nh_);
    tf_ls_ = boost::make_shared<tf2_ros::TransformListener>(tf_buff_);

    ros::Duration(1.0).sleep();

    onInitPostProcess();
  }

  void RectangleDetectionDepth::subscribe()
  {
    if(debug_view_) image_sub_ = it_->subscribe("rgb_img", 1, &RectangleDetectionDepth::imageCallback, this);
    depth_image_sub_ = it_->subscribe("depth_image", 1, &RectangleDetectionDepth::depthImageCallback, this);
    cam_info_sub_ = nh_->subscribe("cam_info", 1, &RectangleDetectionDepth::cameraInfoCallback, this);
  }

  void RectangleDetectionDepth::unsubscribe()
  {
    image_sub_.shutdown();
  }


  void RectangleDetectionDepth::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
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


  void RectangleDetectionDepth::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    rgb_img_ = cv_ptr->image;
  }


  void RectangleDetectionDepth::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    tf2::Transform cam_tf;
    try {
      geometry_msgs::TransformStamped cam_pose_msg = tf_buff_.lookupTransform("world", camera_optical_frame_name_, ros::Time(0), ros::Duration(0.1));
      tf2::convert(cam_pose_msg.transform, cam_tf);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }

    cv::Mat depth_org = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::normalize(depth_org, depth_org, 255, 0, cv::NORM_MINMAX);
    cv::Mat depth_img;
    depth_org.convertTo(depth_img, CV_8UC1);
    depth_img = ~depth_img;

    if(real_size_scale_ == 0) return;

    double object_distance = cam_tf.getOrigin().z() - object_height_;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

    std::vector<std::vector<cv::Point>> contours;
    cv::Mat bin_img;
    cv::threshold(depth_img, bin_img, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);

    //  erosion(to remove lane lines)
    cv::Mat erode_img;
    cv::erode(bin_img, erode_img, cv::Mat(), cv::Point(-1, -1), 3);

    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(erode_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  //  find all contours

    if (contours.size() == 0) {
      NODELET_DEBUG_STREAM("no contours");
      return;  //  no contours -> return
    }

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
      NODELET_DEBUG_STREAM("no valid rects");
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
      sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_img).toImageMsg();
      image_pub_.publish(debug_img_msg);
    }


  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aerial_robot_perception::RectangleDetectionDepth, nodelet::Nodelet);
