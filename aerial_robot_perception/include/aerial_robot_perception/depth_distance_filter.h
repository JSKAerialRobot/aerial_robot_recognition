#pragma once

#include <ros/ros.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <memory>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/assign.hpp>
#include <dynamic_reconfigure/server.h>
#include <aerial_robot_perception/DepthDistanceFilterConfig.h>
#include <limits>

namespace aerial_robot_perception
{
  class DepthDistanceFilter: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    DepthDistanceFilter(): DiagnosticNodelet("DepthDistanceFilter"){}

  private:
    /* ros publisher */
    image_transport::Publisher image_pub_;

    /* ros subscriber */
    image_transport::Subscriber image_sub_;

    /* tf */
    tf2_ros::TransformBroadcaster tf_br_;
    tf2_ros::Buffer tf_buff_;
    std::shared_ptr<tf2_ros::TransformListener> tf_ls_;

    /* image transport */
    std::shared_ptr<image_transport::ImageTransport> it_;

    /* dynamic reconfigure */
    std::shared_ptr<dynamic_reconfigure::Server<aerial_robot_perception::DepthDistanceFilterConfig> > dr_server_;

    double min_depth_;
    double max_depth_;
    double distance_from_ground_;
    bool use_distance_from_ground_;
    bool use_otsu_binarization_;

    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    void reconfigureCallback(aerial_robot_perception::DepthDistanceFilterConfig &config, uint32_t level);

  };
} //namespace aerial_robot_perception
