#pragma once

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_recognition_utils/sensor_model/camera_depth_sensor.h>


namespace aerial_robot_perception
{
  class FarObjectDetection: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    FarObjectDetection(): DiagnosticNodelet("FarObjectDetection"){}

  protected:
    /* ros publisher */
    ros::Publisher target_pos_pub_;
    ros::Publisher red_target_pub_;
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
    int h_min_, h_max_, s_min_, v_min_, center_margin_, min_data_size_, num_of_means_, kmeans_iterations_;

    double real_size_scale_, roll_min_;
    tf2::Matrix3x3 camera_K_inv_;
    tf2::Matrix3x3 camera_K;
    
    double object_distance;
    geometry_msgs::Vector3Stamped obj_pos_msg;
    tf2::Vector3 cam_target_xyz;
    cv::Mat depth_img;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
  };
}
