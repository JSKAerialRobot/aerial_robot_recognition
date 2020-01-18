// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, JSK Lab
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

#include <aerial_robot_perception/line_extraction.h>
#include <algorithm>
#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <vector>
#include <visualization_msgs/Marker.h>

using namespace line_extraction;

namespace aerial_robot_perception
{

  class LaserLineExtraction: public jsk_topic_tools::DiagnosticNodelet
  {

  public:
    LaserLineExtraction(std::string nodelet_name = std::string("LaserLineExtraction")): DiagnosticNodelet(nodelet_name), data_cached_(false) {}

  protected:
    ros::Subscriber scan_sub_;
    ros::Publisher line_pub_;
    ros::Time sensor_timestamp_;

    std::string frame_id_;
    bool verbose_;

    LineExtraction line_extraction_;
    bool data_cached_; // true after first scan used to cache data

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();

    virtual void loadParameters();
    void publish(const std::vector<Line> &lines);
    void cacheData(const sensor_msgs::LaserScan::ConstPtr&);
    virtual void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr&);
  };

}
