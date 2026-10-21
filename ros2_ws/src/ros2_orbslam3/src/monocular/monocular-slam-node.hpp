/*
 * Code from https://github.com/ozandmrz/orb_slam3_ros2_mono_publisher
 */

#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

// System includes
#include <sophus/se3.hpp>
#include <vector>

// ROS2 includes
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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
#include <Atlas.h>

// Local includes
#include "utility.hpp"

class MonocularSLAMNode : public rclcpp::Node
{
public:
    MonocularSLAMNode(ORB_SLAM3::System* pSLAM);
    ~MonocularSLAMNode();

    // Writes the per-frame feature statistics collected during tracking as CSV.
    // Returns the number of frames written.
    size_t save_feature_stats(const std::string& csv_path) const;

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
    using OdometryMsg = nav_msgs::msg::Odometry;

    struct FeatureStat
    {
        double stamp;
        int detected;   // ORB keypoints extracted in the frame
        int matched;    // keypoints matched to map points (inliers)
        int state;      // ORB_SLAM3::Tracking state after tracking the frame
    };

    cv_bridge::CvImagePtr cv_ptrImg_;
    cv::Mat prev_gray_;
    double sharpness_ema_ = 0.0;
    int skipped_frames_ = 0;
    std::vector<FeatureStat> feature_stats_;
    tf2_ros::TransformBroadcaster transform_broadcaster_;
    rclcpp::Time msg_time_;
    ORB_SLAM3::System* SLAM_;
    Sophus::SE3f current_pose_;

    rclcpp::Publisher<PoseStampedMsg>::SharedPtr camera_pose_pub1_;
    rclcpp::Publisher<PoseStampedMsg>::SharedPtr camera_pose_pub2_;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr tracked_mappoints_pub_;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr all_mappoints_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracking_img_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub_;
    rclcpp::Publisher<OdometryMsg>::SharedPtr odom_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_traj_service_;
    rclcpp::Subscription<ImageMsg>::SharedPtr img_sub_;

    bool is_glitchy_frame(const cv::Mat& gray);
    void record_feature_stats(double stamp);
    void grab_data(const ImageMsg::SharedPtr msg);
    void publish_camera_pose(const Sophus::SE3f& Tcw_SE3f, const std_msgs::msg::Header& header);
    void publish_tracked_points(const std::vector<ORB_SLAM3::MapPoint*>& tracked_points, const rclcpp::Time& msg_time);
    void publish_all_points(const std::vector<ORB_SLAM3::MapPoint*>& all_points, const rclcpp::Time& msg_time);
    void publish_tracking_img(cv::Mat image, const rclcpp::Time& msg_time);
    void publish_kf_markers(const std::vector<Sophus::SE3f> kf_poses, const rclcpp::Time& msg_time);
    void publish_odometry(const Sophus::SE3f& Tcw_SE3f, const rclcpp::Time& msg_time);

    sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, const rclcpp::Time& msg_time);

    std::vector<ORB_SLAM3::MapPoint*> GetAllMapPoints();
    std::vector<Sophus::SE3f> GetAllKeyframePoses();

    void setup_services();
    void save_map_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                      std::shared_ptr<std_srvs::srv::Empty::Response> response);
    bool save_traj_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                      std::shared_ptr<std_srvs::srv::Empty::Response> response);
};

#endif // __MONOCULAR_SLAM_NODE_HPP__
