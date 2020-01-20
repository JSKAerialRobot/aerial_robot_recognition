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

#include <aerial_robot_perception/laser_line_extraction_ros.h>
#include <algorithm>
#include <geometry_msgs/TransformStamped.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/topic.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

namespace aerial_robot_perception
{
  class HorizontalBrickDetection: public aerial_robot_perception::LaserLineExtraction
  {
  public:
    HorizontalBrickDetection(): aerial_robot_perception::LaserLineExtraction("HorizontalBrickDetection"){}
  protected:

    void onInit() override;
    void subscribe() override;
    void unsubscribe() override;

    /* ros publisher */
    ros::Publisher objects_marker_pub_;

    /* tf */
    tf2_ros::TransformBroadcaster tf_br_;

    /* brick  */
    double brick_width_, brick_height_;
    double line_estimate_error_;

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr&) override;
    void loadParameters() override;

    void publish(const std::vector<Line> &lines, const std::vector<tf2::Transform> &objects_tf);
  };
};
