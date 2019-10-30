#pragma once

#include <ros/ros.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <numeric>
#include <jsk_recognition_utils/geo/polygon.h>
#include <jsk_recognition_utils/sensor_model/camera_depth_sensor.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>


namespace aerial_robot_perception
{
  class ClutteredBrickDetection: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    ClutteredBrickDetection(): DiagnosticNodelet("ClutteredBrickDetection"){}

  protected:
    void onInit() override;
    void subscribe() override;
    void unsubscribe() override;

    /* ros publisher */
    ros::Publisher target_pos_pub_;
    ros::Publisher target_angle_pub_;
    ros::Publisher area_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher object_marker_pub_;

    /* ros subscriber */
    image_transport::Subscriber depth_image_sub_;
    ros::Subscriber cam_info_sub_;
    ros::Subscriber plane_sub_;

    /* image transport */
    std::shared_ptr<image_transport::ImageTransport> it_;

    /* tf */
    tf2_ros::TransformBroadcaster tf_br_;
    tf2_ros::Buffer tf_buff_;
    std::shared_ptr<tf2_ros::TransformListener> tf_ls_;

    /* ros param */
    bool debug_view_;
    std::string frame_id_, start_message_;
    double optimal_normal_z_; 
    int plane_store_number_, thresh_min_;
    int cut_length_, cut_outlier_;
    double object_width_, object_length_, object_height_, object_area_variance_;
    int object_exist_limit_;
    double left_side_, right_side_, upper_side_, lower_side_, grasp_margin_, roll_min_;
    bool marker_debug_, always_subscribe_;

    double real_size_scale_;
    tf2::Matrix3x3 camera_K_;
    tf2::Matrix3x3 camera_K_inv_;
    jsk_recognition_utils::CameraDepthSensor camdep_;    
    std::vector<jsk_recognition_utils::Vertices> vertice_highest_;

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void planeCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg);
    
  private:
  };
}
