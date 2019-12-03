#include <aerial_robot_perception/far_object_detection.h>

namespace aerial_robot_perception
{

  void FarObjectDetection::onInit()
  {
    DiagnosticNodelet::onInit();

    pnh_->param("debug_view", debug_view_, true);
    pnh_->param("frame_id_", frame_id_, std::string("target"));

    pnh_->param("h_min", h_min_, 20);
    pnh_->param("h_max", h_max_, 150);
    pnh_->param("s_min", s_min_, 128);
    pnh_->param("v_min", v_min_, 128);
    pnh_->param("min_data_size", min_data_size_, 10);
    pnh_->param("num_of_means", num_of_means_, 1);
    pnh_->param("kmeans_iterations", kmeans_iterations_, 100);
    pnh_->param("roll_min", roll_min_, -1.50);
    
    always_subscribe_ = true;
    
    if (debug_view_) image_pub_ = advertiseImage(*pnh_, "debug_image", 1);
    target_pos_pub_ = advertise<geometry_msgs::Vector3Stamped>(*nh_, frame_id_ + std::string("/pos"), 1);
    red_target_pub_ = advertise<geometry_msgs::Point>(*nh_, std::string("red_target/pos"), 1);
    
    it_ = boost::make_shared<image_transport::ImageTransport>(*nh_);
    tf_ls_ = boost::make_shared<tf2_ros::TransformListener>(tf_buff_);
    
    ros::Duration(1.0).sleep();

    onInitPostProcess();
  }

  void FarObjectDetection::subscribe()
  {
    image_sub_ = it_->subscribe("image", 1, &FarObjectDetection::imageCallback, this);
    depth_image_sub_ = it_->subscribe("depth_image", 1, &FarObjectDetection::depthImageCallback, this);
    cam_info_sub_ = nh_->subscribe("cam_info", 1, &FarObjectDetection::cameraInfoCallback, this);  
  }

  void FarObjectDetection::unsubscribe()
  {
    image_sub_.shutdown();
  }

  // kmeans
  struct pt {
    double x;
    double y;
  };

  using DataFrame = std::vector<pt>;

  double square(double value) {
    return value * value;
  }

  double squared_l2_distance(pt first, pt second) {
    return square(first.x - second.x) + square(first.y - second.y);
  }

  int most_freq_cls = 0;


  pt k_means(const DataFrame& data, size_t k, size_t number_of_iterations) {
    static std::random_device seed;
    static std::mt19937 random_number_generator(seed());
    std::uniform_int_distribution<size_t> indices(0, data.size() - 1);
    // Pick centroids as random points from the dataset.
    DataFrame means(k);
    for (auto& cluster : means) {
      cluster = data[indices(random_number_generator)];
    }

    std::vector<size_t> assignments(data.size());
    for (size_t iteration = 0; iteration < number_of_iterations; ++iteration) {
      // Find assignments.
      for (size_t point = 0; point < data.size(); ++point) {
	double best_distance = std::numeric_limits<double>::max();
	size_t best_cluster = 0;
	for (size_t cluster = 0; cluster < k; ++cluster) {
	  const double distance =
	    squared_l2_distance(data[point], means[cluster]);
	  if (distance < best_distance) {
	    best_distance = distance;
	    best_cluster = cluster;
	  }
	}
	assignments[point] = best_cluster;
      }

      // Sum up and count points for each cluster.
      DataFrame new_means(k);
      std::vector<size_t> counts(k, 0);
      for (size_t point = 0; point < data.size(); ++point) {
	const auto cluster = assignments[point];
	new_means[cluster].x += data[point].x;
	new_means[cluster].y += data[point].y;
	counts[cluster] += 1;
      }

      // Divide sums by counts to get new centroids.

      for (size_t cluster = 0; cluster < k; ++cluster) {
	// Turn 0/0 into 0/1 to avoid zero division.
	const auto count = std::max<size_t>(1, counts[cluster]);
	if(count >= data.size() / k){
	  most_freq_cls = cluster;
	}
	means[cluster].x = new_means[cluster].x / count;
	means[cluster].y = new_means[cluster].y / count;
      }
    }

    return means[most_freq_cls];
  }

  
  void FarObjectDetection::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
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


  void FarObjectDetection::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat depth_org = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    depth_org.convertTo(depth_img, CV_32FC1);
  }

  
  void FarObjectDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(real_size_scale_ == 0) return; // no receive camera_info yet.
    
    tf2::Transform cam_tf;
    try{
      geometry_msgs::TransformStamped cam_pose_msg = tf_buff_.lookupTransform("world", "rs_d435_color_optical_frame", ros::Time(0));
      tf2::convert(cam_pose_msg.transform, cam_tf);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }

    tf2::Matrix3x3 cam_tf_rotation(cam_tf.getRotation());
    double roll, pitch, yaw;
    cam_tf_rotation.getRPY(roll, pitch, yaw);  //rpy are Pass by Reference

    //  red object detection
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat src_image = cv_ptr->image;

    // detect the target objects

    cv::Mat hsv_image;
    cvtColor(src_image, hsv_image, cv::COLOR_BGR2HSV);

    DataFrame data;
    geometry_msgs::Point xy_center;

    // find red objects with HSV

    for (int i = 0; i < src_image.rows; i++){
      cv::Vec3b * chan = hsv_image.ptr<cv::Vec3b>(i);
      for (int j = 0; j < src_image.cols; j++){
	cv::Vec3b hsv = chan[j];
	if((hsv[0] < h_min_) || (hsv[0] > h_max_)){
	  if ((hsv[1] > s_min_) && (hsv[2] > v_min_)){
	    pt p = {(double)j, (double)i};
	    data.push_back(p);
	  }
	}
      }
    }

    tf2::Vector3 target_obj_uv;
    
    if(data.size() >= min_data_size_) {
      pt means = k_means(data, num_of_means_, kmeans_iterations_);
      xy_center.x = means.x;
      xy_center.y = means.y;
      xy_center.z = 1.0;

      red_target_pub_.publish(xy_center);
      
      //  debug
      if(debug_view_){
	cv::circle(cv_ptr->image, cv::Point(xy_center.x, xy_center.y), 3, cv::Scalar(0,255,0), 6, 3);
	image_pub_.publish(cv_ptr->toImageMsg());
      }
      
      //  get target point distance
      if(depth_img.cols != 0 && depth_img.rows != 0) object_distance = depth_img.at<float>(xy_center.y, xy_center.x);
      else return;
    }
    
    else return;

    //  too far(depth invalid)
    if(std::isnan(object_distance)) {

      geometry_msgs::Vector3Stamped obj_pos_msg;
      obj_pos_msg.header = msg -> header;
      obj_pos_msg.vector.x = cam_tf.getOrigin().x() + std::cos(pitch+1.57) * 1.0;
      obj_pos_msg.vector.y = cam_tf.getOrigin().y() + std::sin(pitch+1.57) * 1.0;
      obj_pos_msg.vector.z = 0.0;
      target_pos_pub_.publish(obj_pos_msg);

      return;
    }
    
    target_obj_uv.setX(xy_center.x);
    target_obj_uv.setY(xy_center.y);
    target_obj_uv.setZ(xy_center.z);

    tf2::Vector3 object_pos_in_optical_frame = camera_K_inv_ * target_obj_uv * object_distance;

    geometry_msgs::TransformStamped obj_tf_msg;
    obj_tf_msg.header = msg->header;
    obj_tf_msg.header.frame_id = std::string("world");
    obj_tf_msg.child_frame_id = frame_id_;
    obj_tf_msg.transform.translation = tf2::toMsg(cam_tf * object_pos_in_optical_frame);
    obj_tf_msg.transform.rotation.w = 1.0;
    tf_br_.sendTransform(obj_tf_msg);

    geometry_msgs::Vector3Stamped obj_pos_msg;
    obj_pos_msg.header = obj_tf_msg.header;
    obj_pos_msg.vector = obj_tf_msg.transform.translation;

    // go to approach phase
    target_pos_pub_.publish(obj_pos_msg);  
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aerial_robot_perception::FarObjectDetection, nodelet::Nodelet);
