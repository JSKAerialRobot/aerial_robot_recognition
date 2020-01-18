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

#include <aerial_robot_perception/laser_line_extraction_ros.h>

namespace aerial_robot_perception
{

  void LaserLineExtraction::onInit()
  {
    DiagnosticNodelet::onInit();

    loadParameters();
    line_pub_ = advertise<visualization_msgs::Marker>(*nh_, "line_markers", 1);

    onInitPostProcess();
  }

  void LaserLineExtraction::subscribe()
  {
    scan_sub_ = nh_->subscribe("scan", 1, &LaserLineExtraction::laserScanCallback, this);
  }

  void LaserLineExtraction::unsubscribe()
  {
    scan_sub_.shutdown();
  }

  void LaserLineExtraction::loadParameters()
  {
    double bearing_std_dev, range_std_dev, least_sq_angle_thresh, least_sq_radius_thresh,
      max_line_gap, min_line_length, min_range, min_split_dist, outlier_dist;
    int min_line_points;

    ROS_DEBUG("nodename is %s", pnh_->getNamespace().c_str());
    pnh_->param("bearing_std_dev", bearing_std_dev, 1e-3);
    line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);
    ROS_DEBUG("bearing_std_dev: %f", bearing_std_dev);

    pnh_->param("range_std_dev", range_std_dev, 0.02);
    line_extraction_.setRangeVariance(range_std_dev * range_std_dev);
    ROS_DEBUG("range_std_dev: %f", range_std_dev);

    pnh_->param("least_sq_angle_thresh", least_sq_angle_thresh, 1e-4);
    line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
    ROS_DEBUG("least_sq_angle_thresh: %f", least_sq_angle_thresh);

    pnh_->param("least_sq_radius_thresh", least_sq_radius_thresh, 1e-4);
    line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
    ROS_DEBUG("least_sq_radius_thresh: %f", least_sq_radius_thresh);

    pnh_->param("max_line_gap", max_line_gap, 0.4);
    line_extraction_.setMaxLineGap(max_line_gap);
    ROS_DEBUG("max_line_gap: %f", max_line_gap);

    pnh_->param("min_line_length", min_line_length, 0.5);
    line_extraction_.setMinLineLength(min_line_length);
    ROS_DEBUG("min_line_length: %f", min_line_length);

    pnh_->param("min_range", min_range, 0.4);
    line_extraction_.setMinRange(min_range);
    ROS_DEBUG("min_range: %f", min_range);

    pnh_->param("min_split_dist", min_split_dist, 0.05);
    line_extraction_.setMinSplitDist(min_split_dist);
    ROS_DEBUG("min_split_dist: %f", min_split_dist);

    pnh_->param("outlier_dist", outlier_dist, 0.05);
    line_extraction_.setOutlierDist(outlier_dist);
    ROS_DEBUG("outlier_dist: %f", outlier_dist);

    pnh_->param("min_line_points", min_line_points, 9);
    line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
    ROS_DEBUG("min_line_points: %d", min_line_points);

    pnh_->param("verbose", verbose_, false);
    ROS_DEBUG("verbose: %d", verbose_);
  }

  void LaserLineExtraction::publish(const std::vector<Line> &lines)
  {
    visualization_msgs::Marker line_msg;
    line_msg.ns = "line_extraction";
    line_msg.id = 0;
    line_msg.type = visualization_msgs::Marker::LINE_LIST;
    line_msg.scale.x = 0.1;
    line_msg.color.r = 0.0;
    line_msg.color.g = 1.0;
    line_msg.color.b = 0.0;
    line_msg.color.a = 1.0;
    for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
      {
        geometry_msgs::Point p_start;
        p_start.x = cit->getStart()[0];
        p_start.y = cit->getStart()[1];
        p_start.z = 0;
        line_msg.points.push_back(p_start);
        geometry_msgs::Point p_end;
        p_end.x = cit->getEnd()[0];
        p_end.y = cit->getEnd()[1];
        p_end.z = 0;
        line_msg.points.push_back(p_end);
      }
    line_msg.header.frame_id = frame_id_;
    line_msg.header.stamp = sensor_timestamp_;

    line_pub_.publish(line_msg);
  }

  void LaserLineExtraction::cacheData(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
  {
    std::vector<double> bearings, cos_bearings, sin_bearings;
    std::vector<unsigned int> indices;
    for (std::size_t i = 0; i < scan_msg->ranges.size(); ++i)
      {
        const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
        bearings.push_back(b);
        cos_bearings.push_back(cos(b));
        sin_bearings.push_back(sin(b));
        indices.push_back(i);
      }

    line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
    ROS_DEBUG("Data has been cached.");
  }

  void LaserLineExtraction::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
  {
    if (!data_cached_)
      {
        frame_id_ = scan_msg->header.frame_id;
        cacheData(scan_msg);
        data_cached_ = true;
      }

    std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
    line_extraction_.setRangeData(scan_ranges_doubles);

    sensor_timestamp_ = scan_msg->header.stamp;

    std::vector<Line> lines;
    line_extraction_.extractLines(lines, verbose_);

    publish(lines);
  }
} //namespace aerial_robot_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aerial_robot_perception::LaserLineExtraction, nodelet::Nodelet);
