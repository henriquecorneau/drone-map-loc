/*
 * Code from https://github.com/BojanAndonovski71/orb_slam3_ros2 and
 *           https://github.com/ozandmrz/orb_slam3_ros2_mono_publisher
 */

#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

// System includes
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <sophus/se3.hpp>

// ROS2 includes
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>

// ORB SLAM3 includes
#include <System.h>
#include <Frame.h>
#include <Map.h>
#include <Tracking.h>

// Local includes
#include "utility.hpp"

class RGBDSLAMNode : public rclcpp::Node
{
public:
    RGBDSLAMNode(ORB_SLAM3::System* pSLAM);

    ~RGBDSLAMNode();

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    using ImageMsg = sensor_msgs::msg::Image;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
    using OdometryMsg = nav_msgs::msg::Odometry;

    cv_bridge::CvImageConstPtr cv_ptrRGB_;
    cv_bridge::CvImageConstPtr cv_ptrD_;
    tf2_ros::TransformBroadcaster transform_broadcaster_;
    rclcpp::Time msg_time_;
    ORB_SLAM3::System* SLAM_;
    Sophus::SE3f current_pose_;

    rclcpp::Publisher<PoseStampedMsg>::SharedPtr camera_pose_pub_;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr tracked_mappoints_pub_;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr all_mappoints_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracking_img_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub_;
    rclcpp::Publisher<OdometryMsg>::SharedPtr odom_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_trajectory_service_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > rgb_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > sync_approximate_;

    void initialize_subscribers();
    void grab_data(const sensor_msgs::msg::Image::SharedPtr msg_RGB, const sensor_msgs::msg::Image::SharedPtr msg_D);
    void publish_camera_pose(const Sophus::SE3f& Tcw_SE3f, const rclcpp::Time& msg_time);
    void publish_tracked_points(const std::vector<ORB_SLAM3::MapPoint*>& tracked_points, const rclcpp::Time& msg_time);
    void publish_all_points(const std::vector<ORB_SLAM3::MapPoint*>& all_points, const rclcpp::Time& msg_time);
    void publish_tracking_img(cv::Mat image, const rclcpp::Time& msg_time);
    void publish_kf_markers(const std::vector<Sophus::SE3f> vKFposes, const rclcpp::Time& msg_time);
    void publish_odometry(const Sophus::SE3f& Tcw_SE3f, const rclcpp::Time& msg_time);

    sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, const rclcpp::Time& msg_time);

    void setup_services();
    bool save_map_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                      std::shared_ptr<std_srvs::srv::Empty::Response> response);
    bool save_trajectory_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                      std::shared_ptr<std_srvs::srv::Empty::Response> response);
};

#endif
