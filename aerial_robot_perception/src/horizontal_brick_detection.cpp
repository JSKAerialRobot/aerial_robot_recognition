#include <aerial_robot_perception/horizontal_brick_detection.h>

namespace aerial_robot_perception
{
  void HorizontalBrickDetection::onInit()
  {
    LaserLineExtraction::onInit();

    objects_marker_pub_ = advertise<visualization_msgs::MarkerArray>(*nh_, "object_markers", 1);
    onInitPostProcess();
  }

  void HorizontalBrickDetection::subscribe()
  {
    LaserLineExtraction::subscribe();
  }

  void HorizontalBrickDetection::unsubscribe()
  {
    LaserLineExtraction::unsubscribe();
  }

  void HorizontalBrickDetection::loadParameters()
  {
    LaserLineExtraction::loadParameters();

    pnh_->param("line_estimate_error", line_estimate_error_, 0.0);
    ROS_DEBUG("line_estimate_error: %f", line_estimate_error_);

    pnh_->param("brick_width", brick_width_, 0.1);
    ROS_DEBUG("brick_width: %f", brick_width_);

    pnh_->param("brick_height", brick_height_, 0.1);
    ROS_DEBUG("brick_height: %f", brick_height_);

    if(brick_width_ < brick_height_)
      {
        ROS_WARN("brick's width %f should not be shorter than height %f, swap them", brick_width_, brick_height_);
        double temp = brick_height_;
        brick_height_ = brick_width_;
        brick_width_ = temp;
      }
  }

  void HorizontalBrickDetection::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
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

    /* clustering */
    std::vector<std::vector<unsigned int> > range_blobs;
    std::vector<unsigned int> range_blob;
    for (unsigned int i = 0; i < scan_msg->ranges.size (); i++)
      {
        if ( scan_msg->ranges.at(i) < 0 || std::isnan(scan_msg->ranges.at(i)))
          {
            if(range_blob.size() > 0) range_blobs.push_back(range_blob);
            range_blob.clear();
          }
        else
          {
            range_blob.push_back(i);
          }
      }
    if(range_blob.size() > 0)  range_blobs.push_back(range_blob);

    /* line detection and brick fitting */
    std::vector<Line> all_lines;
    std::vector<tf2::Transform> bricks_tf;
    for(const auto& blob : range_blobs)
      {
        line_extraction_.setIndices(blob);
        std::vector<Line> lines;
        line_extraction_.extractLines(lines, false);
        all_lines.insert(all_lines.end(), lines.begin(), lines.end());

        tf2::Quaternion q;
        tf2::Vector3 pos;
        if(lines.size() == 1)
          {
            //ROS_INFO("lines.at(0).length: %f", lines.at(0).length());

            double yaw = atan2(lines.at(0).getEnd().at(1) - lines.at(0).getStart().at(1), lines.at(0).getEnd().at(0) - lines.at(0).getStart().at(0));
            if (lines.at(0).length() > brick_height_ + line_estimate_error_)
              {
                q.setRPY(0, 0, yaw);
                if(scan_msg->ranges.at(lines.at(0).getIndices().at(0)) < scan_msg->ranges.at(lines.at(0).getIndices().back()))
                  pos = tf2::Vector3(lines.at(0).getStart().at(0), lines.at(0).getStart().at(1), 0) + tf2::Matrix3x3(q) * tf2::Vector3(brick_width_ / 2, -brick_height_ / 2, 0);
                else
                  pos = tf2::Vector3(lines.at(0).getEnd().at(0), lines.at(0).getEnd().at(1), 0) + tf2::Matrix3x3(q) * tf2::Vector3(-brick_width_ / 2, -brick_height_ / 2, 0);
              }
            else
              {
                /* we can not determine whether brick_width or brick_height is lines.at(0) */
                /* so choose the short one (i.e., brick_height) as lines.at(0) */

                q.setRPY(0, 0, yaw - M_PI/2);
                if(scan_msg->ranges.at(lines.at(0).getIndices().at(0)) < scan_msg->ranges.at(lines.at(0).getIndices().back()))
                  pos = tf2::Vector3(lines.at(0).getStart().at(0), lines.at(0).getStart().at(1), 0) + tf2::Matrix3x3(q) * tf2::Vector3(brick_width_ / 2, brick_height_ / 2, 0);
                else
                  pos = tf2::Vector3(lines.at(0).getEnd().at(0), lines.at(0).getEnd().at(1), 0) + tf2::Matrix3x3(q) * tf2::Vector3(brick_width_ / 2, -brick_height_ / 2, 0);
              }
            bricks_tf.push_back(tf2::Transform(q, pos));
          }
        else if(lines.size() == 2)
          {
            double yaw1 = atan2(lines.at(0).getEnd().at(1) - lines.at(0).getStart().at(1), lines.at(0).getEnd().at(0) - lines.at(0).getStart().at(0));
            double yaw2 = atan2(lines.at(1).getEnd().at(1) - lines.at(1).getStart().at(1), lines.at(1).getEnd().at(0) - lines.at(1).getStart().at(0));

            if(scan_msg->ranges.at(lines.at(0).getIndices().at(0)) < scan_msg->ranges.at(lines.at(0).getIndices().back()) ||
                   scan_msg->ranges.at(lines.at(1).getIndices().at(0)) > scan_msg->ranges.at(lines.at(1).getIndices().back()))
              {
                ROS_ERROR("Wrong clustering result in laserscan msg, we can not fit brick for lines: [%d, %d] [%d, %d]", lines.at(0).getIndices().at(0), lines.at(0).getIndices().back(), lines.at(1).getIndices().at(0), lines.at(1).getIndices().back());
                continue;
              }

            double delta1 = (sin(yaw2) * (lines.at(1).getStart().at(0) - lines.at(0).getEnd().at(0)) - cos(yaw2) * (lines.at(1).getStart().at(1) - lines.at(0).getEnd().at(1))) / (cos(yaw1) * sin(yaw2) - cos(yaw2) * sin(yaw1));
            double delta2 = (-sin(yaw1) * (lines.at(1).getStart().at(0) - lines.at(0).getEnd().at(0)) + sin(yaw1) * (lines.at(1).getStart().at(1) - lines.at(0).getEnd().at(1))) / (cos(yaw1) * sin(yaw2) - cos(yaw2) * sin(yaw1));

            q.setRPY(0, 0, (yaw1 + yaw2 + M_PI/2) / 2);

            tf2::Vector3 corner = tf2::Vector3(lines.at(0).getEnd().at(0), lines.at(0).getEnd().at(1), 0) + tf2::Matrix3x3(q) * tf2::Vector3(delta1, 0, 0);
            //ROS_INFO("delta1: %f, delta2: %f", delta1, delta2);
            //ROS_INFO("corner: [%f, %f], end1: [%f, %f], start2: [%f, %f]", corner.x(), corner.y(), lines.at(0).getEnd().at(0), lines.at(0).getEnd().at(1), lines.at(1).getStart().at(0), lines.at(1).getStart().at(1));

            if (lines.at(0).length() > brick_height_ + line_estimate_error_)
              {
                pos = corner + tf2::Matrix3x3(q) * tf2::Vector3(-brick_width_ / 2, -brick_height_ / 2, 0);
              }
            else if(lines.at(1).length() > brick_height_ + line_estimate_error_)
              {
                q.setRPY(0, 0, (yaw1 + yaw2 - M_PI/2) / 2);
                pos = corner  + tf2::Matrix3x3(q) * tf2::Vector3(brick_width_ / 2,  -brick_height_ / 2, 0);
              }
            else
              {
                /* select the line which has less gap with brick_height */
                if(brick_height_ - lines.at(0).length()  < brick_height_ - lines.at(1).length())
                  {
                    q.setRPY(0, 0, (yaw1 + yaw2 - M_PI/2) / 2);
                    pos = corner  + tf2::Matrix3x3(q) * tf2::Vector3(brick_width_ / 2,  -brick_height_ / 2, 0);
                  }
                else
                  {
                    pos = corner + tf2::Matrix3x3(q) * tf2::Vector3(-brick_width_ / 2, -brick_height_ / 2, 0);
                  }
              }
            bricks_tf.push_back(tf2::Transform(q, pos));
          }
        else
          {
            // do nothing
          }
      }

    publish(all_lines, bricks_tf);
  }

  void HorizontalBrickDetection::publish(const std::vector<Line> &lines, const std::vector<tf2::Transform> &objects_tf)
  {
    visualization_msgs::MarkerArray obj_markers;

    int i = 1;
    for(const auto& object_tf: objects_tf)
      {
        geometry_msgs::TransformStamped obj_tf_msg;
        obj_tf_msg.header.stamp =  sensor_timestamp_;
        obj_tf_msg.header.frame_id = frame_id_;
        obj_tf_msg.child_frame_id = std::string("brick") + std::to_string(i);
        obj_tf_msg.transform = tf2::toMsg(object_tf);
        tf_br_.sendTransform(obj_tf_msg);

        visualization_msgs::Marker obj_marker;
        obj_marker.header = obj_tf_msg.header;
        obj_marker.ns = obj_tf_msg.child_frame_id;
        obj_marker.id = i;
        obj_marker.action = visualization_msgs::Marker::ADD;
        obj_marker.type = visualization_msgs::Marker::CUBE;
        tf2::toMsg(object_tf, obj_marker.pose);
        obj_marker.scale.x = brick_width_;
        obj_marker.scale.y = brick_height_;
        obj_marker.scale.z = brick_height_;
        obj_marker.color.r = 1.0;
        obj_marker.color.a = 1.0;

        obj_markers.markers.push_back(obj_marker);

        i++;
      }
    objects_marker_pub_.publish(obj_markers);


    LaserLineExtraction::publish(lines);
  }
}  //namespace aerial_robot_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aerial_robot_perception::HorizontalBrickDetection, nodelet::Nodelet);
