// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
n *********************************************************************/

#include <aerial_robot_perception/ground_object_detection.h>

namespace aerial_robot_perception
{
  void GroundObjectDetection::onInit()
  {
    DiagnosticNodelet::onInit();
    /* ros params */
    /** for size filter mode **/
    pnh_->param("contour_area_size", contour_area_size_, 0.2);
    pnh_->param("contour_area_margin", contour_area_margin_, 0.01);
    pnh_->param("contour_area_min_thre", contour_area_min_thre, 0.1);
    pnh_->param("object_height", object_height_, 0.05);
    pnh_->param("debug_view", debug_view_, false);
    pnh_->param("frame_id", frame_id_, std::string("target_object"));
    always_subscribe_ = true; //because of no subscriber

    target_pos_pub_ = advertise<geometry_msgs::Vector3Stamped>(*nh_, frame_id_ + std::string("/pos"), 1);
    if (debug_view_) image_pub_ = advertiseImage(*pnh_, "debug_image", 1);

    it_ = boost::make_shared<image_transport::ImageTransport>(*nh_);
    tf_ls_ = boost::make_shared<tf2_ros::TransformListener>(tf_buff_);

    ros::Duration(1.0).sleep();

    onInitPostProcess();
  }

  void GroundObjectDetection::subscribe()
  {
    image_sub_ = it_->subscribe("image", 1, &GroundObjectDetection::imageCallback, this);
    cam_info_sub_ = nh_->subscribe("camera_info", 1, &GroundObjectDetection::cameraInfoCallback, this);
  }

  void GroundObjectDetection::unsubscribe()
  {
    image_sub_.shutdown();
  }

  void GroundObjectDetection::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    /* the following process is executed once */
    NODELET_DEBUG_STREAM("receive camera info");
    tf2::Matrix3x3 camera_K(msg->K[0], msg->K[1], msg->K[2],
                            msg->K[3], msg->K[4], msg->K[5],
                            msg->K[6], msg->K[7], msg->K[8]);
    camera_K_inv_ = camera_K.inverse();
    real_size_scale_ = msg->K[0] * msg->K[4];

    cam_info_sub_.shutdown();
  }

  void GroundObjectDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(real_size_scale_ == 0) return; // no receive camera_info yet.

    tf2::Transform cam_tf;
    try{
      geometry_msgs::TransformStamped cam_pose_msg = tf_buff_.lookupTransform("world", msg->header.frame_id, msg->header.stamp, ros::Duration(0.2));
      tf2::convert(cam_pose_msg.transform, cam_tf);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      return;
    }

    cv::Mat src_image = cv_bridge::toCvCopy(msg, msg->encoding)->image;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(src_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

    double object_distance = cam_tf.getOrigin().z() - object_height_;
    double object_distance2 = object_distance * object_distance;
    double dist_from_center_min = 1e6;
    double max_contour_area = 0;
    std::vector<cv::Point> target_contour;

    auto calc_position = [](std::vector<cv::Point> contour) {
      cv::Moments contour_moments = cv::moments(contour, true);
      tf2::Vector3 pos;
      pos.setX(contour_moments.m10 / contour_moments.m00);
      pos.setY(contour_moments.m01 / contour_moments.m00);
      return pos;
    };

    for(const auto& contour : contours) {
      double real_contour_area = cv::contourArea(contour) * object_distance2 / real_size_scale_;
      NODELET_DEBUG_STREAM("contour size" << real_contour_area);

      if(contour_area_size_ > 0) {
        /* size filtering mode */
          if(std::abs(real_contour_area - contour_area_size_) < contour_area_margin_) {
            tf2::Vector3 obj_pos = calc_position(contour);
            double dist_from_center = (obj_pos.x() - src_image.cols/2) * (obj_pos.x() - src_image.cols/2) + (obj_pos.y() - src_image.rows/2) * (obj_pos.y() - src_image.rows/2);
            if (dist_from_center < dist_from_center_min) {
              dist_from_center_min = dist_from_center;
              target_contour = contour;
            }
          }
        }
      else {
        /* find maximum size */
        if (max_contour_area < real_contour_area && real_contour_area > contour_area_min_thre) {
          max_contour_area = real_contour_area;
          target_contour = contour;
          dist_from_center_min = 0;
        }
      }
    }

    if (dist_from_center_min != 1e6) {
      if (debug_view_) {
        cv::Mat debug_image = cv::Mat::zeros(src_image.rows, src_image.cols, CV_8U);
        std::vector<std::vector<cv::Point> > draw_contour;
        draw_contour.push_back(target_contour);
        cv::drawContours(debug_image, draw_contour, -1, cv::Scalar(255, 255, 255), -1);
        image_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, debug_image).toImageMsg());
      }

      tf2::Vector3 target_obj_uv = calc_position(target_contour);
      target_obj_uv.setZ(1.0);
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
      target_pos_pub_.publish(obj_pos_msg);
    }
  }
} //namespace aerial_robot_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aerial_robot_perception::GroundObjectDetection, nodelet::Nodelet);
