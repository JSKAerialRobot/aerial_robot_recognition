#include <aerial_robot_perception/rectangle_detection_depth.h>

namespace aerial_robot_perception
{

  void RectangleDetectionDepth::onInit()
  {
    DiagnosticNodelet::onInit();

    pnh_->param("debug_view", debug_view_, true);
    pnh_->param("frame_id", frame_id_, std::string("target"));
    pnh_->param("image_width", image_width_, 1280);
    pnh_->param("image_height", image_height_, 720);
    pnh_->param("lowest_margin", lowest_margin_, 10);
    pnh_->param("object_height", object_height_, 0.20);
    pnh_->param("target_object_area", target_object_area_, 0.06);
    pnh_->param("target_object_area_margin", target_object_area_margin_, 0.02);
    always_subscribe_ = true;

    if (debug_view_) image_pub_ = advertiseImage(*pnh_, "debug_image", 1);
    target_pub_ = advertise<geometry_msgs::TransformStamped>(*nh_, frame_id_, 1);
    
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
    real_size_scale_ = msg->K[0] * msg->K[4];
    cam_info_sub_.shutdown();
  }


  void RectangleDetectionDepth::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    rgb_img = cv_ptr->image;
  }

  
  void RectangleDetectionDepth::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    tf2::Transform cam_tf;
    try{
      geometry_msgs::TransformStamped cam_pose_msg = tf_buff_.lookupTransform("world", "rs_d435_color_optical_frame", ros::Time(0));
      tf2::convert(cam_pose_msg.transform, cam_tf);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }

    cv::Mat depth_org = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::normalize(depth_org, depth_org, 255, 0, cv::NORM_MINMAX);
    depth_org.convertTo(depth_img, CV_8UC1);
    depth_img = ~depth_img;

    if(real_size_scale_ == 0) return;

    tf2::Matrix3x3 cam_tf_rotation(cam_tf.getRotation());
    double roll, pitch, yaw;
    cam_tf_rotation.getRPY(roll, pitch, yaw);

    object_distance = cam_tf.getOrigin().z() - object_height_;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

    geometry_msgs::Point rect_center_msg;
    tf2::Vector3 target_obj_uv;
    float target_angle;
    cv::Point2f vertices[4];
    int rect_center_x = 0;
    int rect_center_y = 0;

    std::vector<std::vector<cv::Point>> contours;
    cv::Mat bin_img;
    cv::threshold(depth_img, bin_img, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);

    //  erosion(to remove lane lines)
    cv::Mat erode_img;
    cv::erode(bin_img, erode_img, cv::Mat(), cv::Point(-1, -1), 3);  
    
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(erode_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  //  find all contours

    if(contours.size() == 0) return;  //  no contours -> return

    for(int i=0; i<contours.size(); i++){
      cv::Mat input_points;
      cv::Mat(contours[i]).convertTo(input_points, CV_32F);
      cv::RotatedRect rects =  cv::minAreaRect(input_points);  //  detect rectangles(with degree)

      cv::Point2f vertices_tmp[4];
      rects.points(vertices_tmp);  //  get rect vertices

      //  check whether target object or not based on its area
      std::vector<tf2::Vector3> rect_points;
      for(int j=0; j<3; j++){
	tf2::Vector3 rect_point_img;
	rect_point_img.setX(vertices_tmp[j].x);
	rect_point_img.setY(vertices_tmp[j].y);
	rect_point_img.setZ(1.0);

	tf2::Vector3 rect_point = camera_K_inv_ * rect_point_img * object_distance;
	rect_points.push_back(rect_point);
      }

      double rect_area = std::sqrt(std::pow((rect_points[0].x()-rect_points[1].x()), 2.0)+std::pow((rect_points[0].y()-rect_points[1].y()), 2.0)) * std::sqrt(std::pow((rect_points[1].x()-rect_points[2].x()), 2.0)+std::pow((rect_points[1].y()-rect_points[2].y()), 2.0));

      rect_points.erase(rect_points.begin(), rect_points.end());
      if(std::abs(rect_area-target_object_area_) > target_object_area_margin_) continue;

      //  draw all target rects(for debug)
      if(debug_view_){
	for (int j=0; j<4; j++){
	  cv::line(rgb_img, vertices_tmp[j], vertices_tmp[(j+1)%4], cv::Scalar(255,0,0), 10);
	}
      }
      
      int shift_x = std::abs(rects.center.x - 0.5*image_width_);
      int shift_x_before = std::abs(rect_center_x - 0.5*image_width_);
      int change_y = std::abs(rects.center.y - rect_center_y);

      //  detect rect which is most near to the center in x axis and the lowest in the y axis
      if((shift_x < shift_x_before && change_y < std::abs(shift_x - shift_x_before)) || (rect_center_y < rects.center.y && std::abs(shift_x - shift_x_before) < change_y)){

	//  exclude too low rect in y axis
	if((image_height_ - vertices_tmp[0].y) < lowest_margin_) continue;
	if((image_height_ - vertices_tmp[1].y) < lowest_margin_) continue;
	if((image_height_ - vertices_tmp[2].y) < lowest_margin_) continue;
	if((image_height_ - vertices_tmp[3].y) < lowest_margin_) continue;

	rect_center_x = rects.center.x;
	rect_center_y = rects.center.y;

	target_angle = (-1) * rects.angle * M_PI / 180 + 0.78;
	rects.points(vertices);
      }
    }
    
    //  draw target rect(for debug)
    if(debug_view_){
      for (int i=0; i<4; i++){
	cv::line(rgb_img, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0), 10);
      }
      cv::imwrite("/home/kuromiya/result.png", rgb_img);
      sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_img).toImageMsg();
      image_pub_.publish(rgb_msg);
    }

    //  target point(center of rect)
    target_obj_uv.setX(rect_center_x);
    target_obj_uv.setY(rect_center_y);
    target_obj_uv.setZ(1.0);
    
    tf2::Vector3 object_pos_in_optical_frame = camera_K_inv_ * target_obj_uv * object_distance;

    geometry_msgs::TransformStamped obj_tf_msg;
    obj_tf_msg.header = msg->header;
    obj_tf_msg.header.frame_id = std::string("rs_d435_color_optical_frame");
    obj_tf_msg.child_frame_id = frame_id_;
    obj_tf_msg.transform.translation = tf2::toMsg(object_pos_in_optical_frame);
    obj_tf_msg.transform.rotation = tf2::toMsg(tf2::Quaternion(target_angle, pitch, roll));
    tf_br_.sendTransform(obj_tf_msg);

    target_pub_.publish(obj_tf_msg);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aerial_robot_perception::RectangleDetectionDepth, nodelet::Nodelet);
