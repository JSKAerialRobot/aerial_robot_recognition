#pragma once

#include <iostream>
#include <iomanip>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <aerial_robot_perception/RectangleDetectionConfig.h>

namespace aerial_robot_perception
{
  class RectangleDetection: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    RectangleDetection(): DiagnosticNodelet("RectangleDetectionDepth"){}

  protected:
    /* ros publisher */
    ros::Publisher target_pub_;
    image_transport::Publisher debug_image_pub_;

    /* ros subscriber */
    image_transport::Subscriber rgb_image_sub_;
    image_transport::Subscriber mask_image_sub_;
    ros::Subscriber cam_info_sub_;

    /* image transport */
    std::shared_ptr<image_transport::ImageTransport> it_;

    /* tf */
    tf2_ros::TransformBroadcaster tf_br_;
    tf2_ros::Buffer tf_buff_;
    std::shared_ptr<tf2_ros::TransformListener> tf_ls_;

    /* dynamic reconfigure */
    std::shared_ptr<dynamic_reconfigure::Server<aerial_robot_perception::RectangleDetectionConfig> > dr_server_;

    /* ros param */
    bool debug_view_;
    std::string frame_id_;

    double real_size_scale_;
    tf2::Matrix3x3 camera_K_inv_;
    tf2::Matrix3x3 camera_K;

    int lowest_margin_, image_width_, image_height_;
    double object_height_, target_object_area_, target_object_area_margin_;

    cv::Mat rgb_img_;
    std::string rgb_img_encoding_;

    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void maskImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void reconfigureCallback(aerial_robot_perception::RectangleDetectionConfig &config, uint32_t level);

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
  };
}
