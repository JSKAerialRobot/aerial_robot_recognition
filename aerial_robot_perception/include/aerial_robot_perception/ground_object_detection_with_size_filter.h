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
 *********************************************************************/

#pragma once

#include <ros/ros.h>
#include <ros/topic.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>

#include <algorithm>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace aerial_robot_perception
{

  class GroundObjectDetectionWithSizeFilter: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    GroundObjectDetectionWithSizeFilter(): DiagnosticNodelet("GroundObjectDetectionWithSizeFilter"), real_size_scale_(0) {}

  protected:
    /* ros publisher */
    ros::Publisher target_pos_pub_;
    image_transport::Publisher image_pub_; //for debug

    /* ros subscriber */
    image_transport::Subscriber image_sub_;
    ros::Subscriber cam_info_sub_;

    /* image transport */
    boost::shared_ptr<image_transport::ImageTransport> it_;

    /* tf */
    tf2_ros::TransformBroadcaster tf_br_;
    tf2_ros::Buffer tf_buff_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_ls_;

    /* ros param */
    double contour_area_size_, contour_area_margin_;
    double object_height_;
    bool debug_view_;
    std::string frame_id_;

    double real_size_scale_;
    tf2::Matrix3x3 camera_K_inv_;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
  };

} //namespace aerial_robot_perception
