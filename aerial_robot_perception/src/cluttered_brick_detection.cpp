#include <aerial_robot_perception/cluttered_brick_detection.h>

namespace aerial_robot_perception
{

  void ClutteredBrickDetection::onInit()
  {
    DiagnosticNodelet::onInit();
    
    pnh_->param("debug_view", debug_view_, true);
    pnh_->param("marker_debug", marker_debug_, true);
    pnh_->param("frame_id_", frame_id_, std::string("target_object"));
    
    pnh_->param("optimal_normal_z", optimal_normal_z_, 0.8);
    pnh_->param("plane_store_number", plane_store_number_, 5);
    pnh_->param("thresh_min", thresh_min_, 50);
    pnh_->param("cut_length", cut_length_, 4);
    pnh_->param("object_width", object_width_, 0.2);
    pnh_->param("object_length", object_length_, 0.3);
    pnh_->param("object_height", object_height_, 0.2);
    pnh_->param("object_area_variance", object_area_variance_, 0.010);
    pnh_->param("object_exist_limit", object_exist_limit_, 500);
    pnh_->param("left_side", left_side_, 1.0);
    pnh_->param("right_side", right_side_, 3.0);
    pnh_->param("upper_side", upper_side_, 1.0);
    pnh_->param("lower_side", lower_side_, 3.0);
    pnh_->param("grasp_margin", grasp_margin_, 0.70);
    pnh_->param("roll_min", roll_min_, 3.14);
    pnh_->param("always_subscribe", always_subscribe_, true);

    target_pos_pub_ = advertise<geometry_msgs::Vector3Stamped>(*nh_, frame_id_ + std::string("/pos"), 1);
    target_angle_pub_ = advertise<std_msgs::Float64>(*nh_, frame_id_ + std::string("/angle"), 1);
    object_marker_pub_ = advertise<visualization_msgs::Marker>(*nh_, frame_id_ + std::string("/visualize"), 1);
    marker_pub_ = advertise<visualization_msgs::Marker>(*nh_, "marker", 1);
    
    it_ = std::make_shared<image_transport::ImageTransport>(*nh_);
    tf_ls_ = std::make_shared<tf2_ros::TransformListener>(tf_buff_);

    ros::Duration(1.0).sleep();

    onInitPostProcess();
  }

  void ClutteredBrickDetection::subscribe()
  {
    cam_info_sub_ = nh_->subscribe("cam_info", 1, &ClutteredBrickDetection::cameraInfoCallback, this);
    plane_sub_ = nh_->subscribe("polygon_array", 1, &ClutteredBrickDetection::planeCallback, this);
    depth_image_sub_ = it_->subscribe("depth_image", 1, &ClutteredBrickDetection::depthImageCallback, this);

  }

  void ClutteredBrickDetection::unsubscribe()
  {
    depth_image_sub_.shutdown();
  }
  
  void ClutteredBrickDetection::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    /* the following process is executed once */
    NODELET_DEBUG_STREAM("receive camera info");
    camdep_.setCameraInfo(*msg);
    cam_info_sub_.shutdown();
  }

  
  void ClutteredBrickDetection::planeCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg)
  /*  get high and straight plane(candidate)  */ 
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

    geometry_msgs::PolygonStamped polygons_stamped[sizeof(msg->polygons) / sizeof(msg->polygons[0])];
    geometry_msgs::Polygon image_plane;

    tf2::Matrix3x3 cam_tf_rotation(cam_tf.getRotation());

    std::vector<double> highest_z;
    tf2::Vector3 highest_normal_world;

    for(const auto polygon_stamped : msg->polygons)
      {
	geometry_msgs::Polygon polygon = polygon_stamped.polygon;

	jsk_recognition_utils::Vertices vertice;

	//  store plane points
	for (size_t i = 0; i < polygon.points.size(); i++) {
	  Eigen::Vector3f v;
	  jsk_recognition_utils::pointFromXYZToVector<geometry_msgs::Point32, Eigen::Vector3f>(polygon.points[i], v);
	  vertice.push_back(v);
	}

	//  get normal
	jsk_recognition_utils::Polygon area_polygon(vertice);
	Eigen::Vector3f normal_cam = area_polygon.getNormalFromVertices();
	tf2::Vector3 normal;
	normal.setX(normal_cam(0));
	normal.setY(normal_cam(1));
	normal.setZ(normal_cam(2));
	tf2::Vector3 normal_world = cam_tf_rotation * normal;

	//  get plane center
	Eigen::Vector3f center_cam = area_polygon.centroid();
	tf2::Vector3 center;
	center.setX(center_cam(0));
	center.setY(center_cam(1));
	center.setZ(center_cam(2));
	  
	area_polygon.fromROSMsg(polygon);

	tf2::Vector3 center_world = cam_tf * center;

	// store 5 vetice candidates

	// judge whether the detected plane is truly one of the surface of the target objects
	if(std::abs(area_polygon.area() - object_width_ * object_width_) < object_area_variance_ || std::abs(area_polygon.area() - object_width_ * object_height_) < object_area_variance_){
	  // detect only upper surface
	  if(std::abs(normal_world.z()) > optimal_normal_z_){
	    if(vertice_highest_.size() < plane_store_number_){
	      vertice_highest_.push_back(vertice);
	      highest_z.push_back(center_world.z());
	    }
	    else{
	      //  replace with higher plane
	      int z_count = 0;
	      for(const auto z : highest_z){
		if(center_world.z() > z){
		  highest_z.erase(highest_z.begin() + z_count);
		  vertice_highest_.erase(vertice_highest_.begin() + z_count);
		  highest_z.push_back(center_world.z());
		  vertice_highest_.push_back(vertice);
		  break;
		}
		z_count++;
	      }
	    }
	  }
	}
      }
    }

  void ClutteredBrickDetection::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
    
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

    tf2::Matrix3x3 cam_tf_rotation(cam_tf.getRotation());
    double roll, pitch, yaw;
    cam_tf_rotation.getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
    if(std::abs(roll) < roll_min_) return;
   
    cv::Mat depth_img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    cv::normalize(depth_img, depth_img, 255, 0, cv::NORM_MINMAX);
    cv::Mat depth;
    depth_img.convertTo(depth, CV_8UC1);
    
    double target_angle_deg;

    float target_cam_x, target_cam_y, target_cam_z;  //  target pos on cam_tf
    
    if(vertice_highest_.size() != 0){
      for(const auto v_highest : vertice_highest_){
	jsk_recognition_utils::Polygon line_img_polygon(v_highest);

	target_cam_x = line_img_polygon.centroid()(0);
	target_cam_y = line_img_polygon.centroid()(1);
	target_cam_z = line_img_polygon.centroid()(2);
	
	cv::Mat src(depth_img.cols, depth_img.rows, CV_8UC3, cv::Scalar(255, 255, 255));          
	line_img_polygon.drawLineToImage(camdep_, src, cv::Scalar(0, 255, 0), 1);
	if(depth_img.cols != 0 && depth_img.rows != 0){

	  std_msgs::Float64 target_angle;
	  /*  space detector */

	  // for contour detection
	  cv::Mat gray;
	  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
	  cv::Mat bw;
	  cv::threshold(gray, bw, thresh_min_, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	  std::vector<cv::Point> contour;

	  
	  // get contour points
	  for(int y = 0; y < src.rows; y++){
	    cv::Vec3b * sr = src.ptr<cv::Vec3b>(y);
	    for(int x =0; x < src.cols; x++){
	      cv::Vec3b s = sr[x];
	      if((s[0] == 0) && (s[1] == 255) == (s[2] == 0)){
		cv::Point point;
		point.x = x;
		point.y = y;
		contour.push_back(point);
	      }
	    }
	  }

	  cv::RotatedRect box;

	  //  get rect (if impossible -> fail)
	  try{
	    box = cv::minAreaRect(contour);
	  }
	  catch(cv::Exception &ex){
	    ROS_WARN("%s", ex.what());
	    //  cannot grasp
	    target_angle_deg = 0.0;

	    continue;
	  }

	  float angle = box.angle;

	  cv::Mat thresh;
	  int length;

	  int center_x = box.center.x;
	  int center_y = box.center.y;

	  //  rotate image so that the contour is horizontal to the image 

	  cv::Mat transform_m = cv::getRotationMatrix2D(cv::Point(depth.cols/2, depth.rows/2), angle, 1);

	  cv::Mat rotate;
	  cv::warpAffine(depth, rotate, transform_m, depth.size(), CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));

	  
	  cv::Mat transform_mm;
	  cv::invertAffineTransform(transform_m, transform_mm);
	  
	  int center_x_rotated = transform_m.at<double>(0, 0) * center_x + transform_m.at<double>(0, 1) * center_y + transform_m.at<double>(0, 2);

	  int center_y_rotated = transform_m.at<double>(1, 0) * center_x + transform_m.at<double>(1, 1) * center_y + transform_m.at<double>(1, 2);	    

	  angle = -angle;  //  reverse(camera -> hydrus)

	  int width = box.size.width;
	  int height = box.size.height;
	  
	  length = std::max(width, height);
 

	  int x_min = center_x_rotated - 2*length;
	  int x_max = center_x_rotated + 2*length;
	  int y_min = center_y_rotated - 2*length;
	  int y_max = center_y_rotated + 2*length;

	  if(x_min < 0){
	    x_min = 0;
	  }
  
	  if(x_max >= rotate.cols){
	    x_max = rotate.cols-1;
	  }

	  if(y_min < 0){
	    y_min = 0;
	  }

	  if(y_max >= rotate.rows){
	    y_max = rotate.rows-1;
	  }
  
	  cv::Rect rotate_area(x_min, y_min, x_max-x_min, y_max-y_min);
	  cv::Mat new_rotate = rotate(rotate_area);

	  x_min += (rotate.cols/2 - center_x_rotated);
	  x_max += (rotate.cols/2 - center_x_rotated);
	  y_min += (rotate.rows/2 - center_y_rotated);
	  y_max += (rotate.rows/2 - center_y_rotated);
  
 	  cv::Mat cut(cv::Size(rotate.cols, rotate.rows), CV_8UC1, cv::Scalar(255));
	  cv::Rect cut_area(x_min, y_min, x_max-x_min, y_max-y_min);
	  cv::Mat new_cut = cut(cut_area);
	  new_rotate.copyTo(new_cut);
	  
	  cv::Mat cutt = cut(cv::Rect((depth.cols/2 - (cut_length_ / 2) * length), (depth.rows/2 - (cut_length_ / 2) *length), 4*length, 4*length));


	  int c = 0;  // threshold pixel value(around the center of contour)
	  for(int i = 2 *length - 10; i < 2 * length + 10; i++){
	    uchar* th = cutt.ptr<uchar>(i);
	    for(int j = 2 * length - 10; j < 2 * length + 10; j++){
	      uchar t = th[j];
	      if(t != 0 && t != 255){  // exclude outliers
		c = t;
		break;
	      }
	    }
	  }
        
	  cv::threshold(cutt, thresh, c + 255 / cam_tf.getOrigin().z() * object_height_, 255, cv::THRESH_BINARY);

	  // check whether hydrus can grasp the object

	  int left_upper_count = 0;
	  int left_lower_count = 0;
	  int right_upper_count = 0;
	  int right_lower_count = 0;

	  int left_count = 0;
	  int right_count = 0;
	  int down_count = 0;
	  int up_count = 0;
  
	  int black_count = 0;
  
	  for(int i = 0; i < thresh.rows; i++){ // y
	    uchar * th = thresh.ptr<uchar>(i);
	    for(int j = 0; j < thresh.cols; j++){ // x
	      uchar t = th[j];
      
	      if(int(t) == 0){
		black_count++;
		// detect which direction is best(with a little bit margin)
      
		if(i < upper_side_ * length && j < left_side_ * length){
		  left_upper_count++;
		}
		else if(i > lower_side_ * length && j < left_side_ * length){
		  left_lower_count++;
		}
		else if(i < upper_side_ * length && j > right_side_ * length){
		  right_upper_count++;
		}
		else if(i > lower_side_ * length && j > right_side_* length){
		  right_lower_count++;
		}
		else if (j < left_side_ * length){
		  left_count++;
		}
		else if (j > right_side_ * length){

		  right_count++;
		}
		else if (i > lower_side_ * length){
		  down_count++;
		}
		else if (i < upper_side_ * length){
		  up_count++;
		}	
	      }
	    }
	  }

	  std::vector<int> possible_answer; // 0:left, 1:right, 2:up, 3:down
	  possible_answer.push_back(0);
	  possible_answer.push_back(1);
	  possible_answer.push_back(2);
	  possible_answer.push_back(3);
  
	  int delete_times = 0;
  
	  if(left_count > object_exist_limit_){
	    if(delete_times == 0){
	      possible_answer.erase(possible_answer.begin() + 0);
	      possible_answer.erase(possible_answer.begin() + 1);
	      possible_answer.erase(possible_answer.begin() + 1);
	    }
	    delete_times++;
	  }

  
	  if(right_count > object_exist_limit_){
	    if(delete_times == 0){
	      possible_answer.erase(possible_answer.begin() + 1);
	      possible_answer.erase(possible_answer.begin() + 1);
	      possible_answer.erase(possible_answer.begin() + 1);
	    }
	    delete_times++;
	  }

	  if(up_count > object_exist_limit_){
	    if(delete_times == 0){
	      possible_answer.erase(possible_answer.begin() + 0);
	      possible_answer.erase(possible_answer.begin() + 0);
	      possible_answer.erase(possible_answer.begin() + 0);
	    }
	    delete_times++;
	  }
  
	  if(down_count > object_exist_limit_){
	    if(delete_times == 0){
	      possible_answer.erase(possible_answer.begin() + 0);
	      possible_answer.erase(possible_answer.begin() + 0);
	      possible_answer.erase(possible_answer.begin() + 1);
	    }
	    delete_times++;    
	  }
    
	  if(delete_times > 1){ // multiple object exists -> unable to grasp
	    continue;
	  }
		 

	  else if(left_lower_count < object_exist_limit_ && right_lower_count < object_exist_limit_ && std::find(possible_answer.begin(), possible_answer.end(), 3) != possible_answer.end()){
	    //  down
	    target_angle_deg = angle+90;
	   
	    break;
	  }

	  else if(left_upper_count < object_exist_limit_ && right_upper_count < object_exist_limit_ && std::find(possible_answer.begin(), possible_answer.end(), 2) != possible_answer.end()){
	    //  up
	    target_angle_deg = angle+270;
    
	    break;
	  }

	  else if(left_upper_count < object_exist_limit_ && left_lower_count < object_exist_limit_ && std::find(possible_answer.begin(), possible_answer.end(), 0) != possible_answer.end()){
	    //  left
	    target_angle_deg = angle;
	    
	    break;
	  }

	  
	  else if(right_upper_count < object_exist_limit_ && right_lower_count < object_exist_limit_ &&std::find(possible_answer.begin(), possible_answer.end(), 1) != possible_answer.end()){
	    //  right
	    target_angle_deg = angle+180;

	    break;

	  }

	  else{
	    // unable to grasp
	    target_angle.data = 0.0;
	  
	    continue;
	  }
	}
      }
      
      //  publish target pos and yaw
      if(target_angle_deg != 0.0){

	tf2::Vector3 box_xyz;  // target position on cam_tf
	box_xyz.setX(target_cam_x);
	box_xyz.setY(target_cam_y);
	box_xyz.setZ(target_cam_z);

	geometry_msgs::Vector3Stamped box_pos_msg;
	box_pos_msg.header = msg->header;
	box_pos_msg.vector = tf2::toMsg(cam_tf * box_xyz);

	double target_angle_rad = target_angle_deg * M_PI / 180;

	double cam_angle = yaw;

	double target_angle = cam_angle + target_angle_rad;
	
	//  target angle -> publish
	std_msgs::Float64 angle;
	angle.data = target_angle;
	target_angle_pub_.publish(angle);
	
	double target_x = box_pos_msg.vector.x - grasp_margin_ * std::cos(target_angle);
	double target_y = box_pos_msg.vector.y - grasp_margin_ * std::sin(target_angle);
	double target_z = box_pos_msg.vector.z;                                 

	geometry_msgs::Vector3Stamped tar_pos_msg;
	tar_pos_msg.header = msg->header;
	tar_pos_msg.vector.x = target_x;
	tar_pos_msg.vector.y = target_y;
	tar_pos_msg.vector.z = target_z;
	
	//  target position -> publish
	target_pos_pub_.publish(tar_pos_msg);
       
	//  pub marker(for debug)
	if(marker_debug_){
	  visualization_msgs::Marker marker_obj_msg;
	  marker_obj_msg.header.frame_id = "/world";
	  marker_obj_msg.header.stamp = msg->header.stamp;
	  marker_obj_msg.ns = "basic_shapes";

	  marker_obj_msg.type = visualization_msgs::Marker::CUBE;
	  marker_obj_msg.action = visualization_msgs::Marker::ADD;
	  marker_obj_msg.scale.x = object_width_;
	  marker_obj_msg.scale.y = object_height_;
	  marker_obj_msg.scale.z = object_length_;
	  marker_obj_msg.pose.position.x = box_pos_msg.vector.x;
	  marker_obj_msg.pose.position.y = box_pos_msg.vector.y;
	  marker_obj_msg.pose.position.z = box_pos_msg.vector.z - object_length_ / 2;
	  marker_obj_msg.pose.orientation =  tf::createQuaternionMsgFromYaw(target_angle);

	  marker_obj_msg.color.g = 1.0f;
	  marker_obj_msg.color.a = 1.0f;
	  object_marker_pub_.publish(marker_obj_msg);

	  visualization_msgs::Marker marker;
	  marker.header.frame_id = "/world";
	  marker.header.stamp = ros::Time(0);
	  marker.ns = "basic_shapes";
	  marker.id = 0;

	  marker.type = visualization_msgs::Marker::ARROW;
	  marker.action = visualization_msgs::Marker::ADD;
	  marker.lifetime = ros::Duration();

	  marker.scale.x = 0.5;
	  marker.scale.y = 0.05;
	  marker.scale.z = 0.05;
	  marker.pose.position.x = target_x;
	  marker.pose.position.y = target_y;
	  marker.pose.position.z = target_z;
	  marker.pose.orientation = tf::createQuaternionMsgFromYaw(target_angle);
	  marker.color.b = 1.0f;
	  marker.color.a = 1.0f;
	  marker_pub_.publish(marker);
	}
      }
    }
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aerial_robot_perception::ClutteredBrickDetection, nodelet::Nodelet);
