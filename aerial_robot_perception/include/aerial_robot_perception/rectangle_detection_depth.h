#pragma once

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <limits>
#include <random>
#include <cmath>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
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
#include <jsk_recognition_utils/sensor_model/camera_depth_sensor.h>

namespace aerial_robot_perception
{
  class RectangleDetectionDepth: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    RectangleDetectionDepth(): DiagnosticNodelet("RectangleDetectionDepth"){}

  protected:
    /* ros publisher */
    ros::Publisher target_pub_;
    image_transport::Publisher image_pub_;

    /* ros subscriber */
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_image_sub_;
    ros::Subscriber cam_info_sub_;

    /* image transport */
    boost::shared_ptr<image_transport::ImageTransport> it_;

    /* tf */
    tf2_ros::TransformBroadcaster tf_br_;
    tf2_ros::Buffer tf_buff_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_ls_;

    /* ros param */
    bool debug_view_;
    std::string frame_id_;

    double real_size_scale_;
    tf2::Matrix3x3 camera_K_inv_;
    tf2::Matrix3x3 camera_K;

    int lowest_margin_, image_width_, image_height_;
    double object_height_, target_object_area_, target_object_area_margin_;

    cv::Mat rgb_img_;
    std::string camera_optical_frame_name_;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
  };
}
