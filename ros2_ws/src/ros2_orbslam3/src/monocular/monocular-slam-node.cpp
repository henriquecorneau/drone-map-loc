/*
 * Code from https://github.com/ozandmrz/orb_slam3_ros2_mono_publisher
 */

// System includes
#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iomanip>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ROS2 includes
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>

// Local includes
#include "monocular-slam-node.hpp"

using std::placeholders::_1;

MonocularSLAMNode::MonocularSLAMNode(ORB_SLAM3::System* pSLAM)
: Node("ORB_SLAM3_ROS2"),
  transform_broadcaster_(this)
{
    SLAM_ = pSLAM;

    const std::string image_topic = this->declare_parameter<std::string>("image_topic", "/tello/camera");

    // Image subscriber
    img_sub_ = this->create_subscription<ImageMsg>(
        image_topic,
        rclcpp::QoS(10), 
        std::bind(&MonocularSLAMNode::grab_data, this, std::placeholders::_1)
    );
    
    //TODO: check the reason to have two camera pose publishers
    // First camera pose publisher for grab_data
    camera_pose_pub1_ = this->create_publisher<PoseStampedMsg>("/pose_orb1", rclcpp::QoS(10));
    
    // Second camera pose publisher for publish_camera_pose
    camera_pose_pub2_ = this->create_publisher<PoseStampedMsg>("/pose_orb2", rclcpp::QoS(10));

    // Tracked map points publisher
    tracked_mappoints_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/tracked_mappoints", rclcpp::QoS(10));

    // All map points publisher
    all_mappoints_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/all_mappoints", rclcpp::QoS(10));
    
    // Tracking image publisher
    tracking_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/tracking_image", rclcpp::QoS(10));
    
    kf_markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        std::string(this->get_name()) + "/kf_markers", rclcpp::QoS(1000));
        
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));

    current_pose_ = Sophus::SE3f();
    
    setup_services();
}

MonocularSLAMNode::~MonocularSLAMNode()
{
    SLAM_->Shutdown();
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

bool MonocularSLAMNode::is_glitchy_frame(const cv::Mat& gray)
{
    cv::Mat lap;
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    const double sharpness = stddev[0] * stddev[0];

    bool glitchy = false;
    if(sharpness_ema_ > 1.0 && sharpness < 0.35 * sharpness_ema_)
        glitchy = true;

    if(!prev_gray_.empty() && prev_gray_.size() == gray.size())
    {
        cv::Mat diff;
        cv::absdiff(gray, prev_gray_, diff);
        if(cv::mean(diff)[0] > 45.0)
            glitchy = true;
    }

    if(glitchy && skipped_frames_ < 8)
    {
        skipped_frames_++;
        return true;
    }

    skipped_frames_ = 0;
    sharpness_ema_ = (sharpness_ema_ < 1.0) ? sharpness : 0.9 * sharpness_ema_ + 0.1 * sharpness;
    prev_gray_ = gray.clone();
    return false;
}

void MonocularSLAMNode::record_feature_stats(double stamp, const Sophus::SE3f& T_camera_world)
{
    // System's copies of the current frame; outliers are already NULL.
    const int detected = static_cast<int>(SLAM_->GetTrackedKeyPointsUn().size());
    const std::vector<ORB_SLAM3::MapPoint*> map_points = SLAM_->GetTrackedMapPoints();
    const int matched = static_cast<int>(
        std::count_if(map_points.begin(), map_points.end(),
                      [](ORB_SLAM3::MapPoint* mp) { return mp != nullptr; }));

    FeatureStat stat{stamp, detected, matched, SLAM_->GetTrackingState(), false, 0, Sophus::SE3f()};
    if(stat.state == ORB_SLAM3::Tracking::OK || stat.state == ORB_SLAM3::Tracking::OK_KLT)
    {
        stat.T_world_camera = T_camera_world.inverse();
        stat.tracked = stat.T_world_camera.translation().allFinite();
        // Relocalization may switch the active map of a multi-map atlas.
        stat.map_id = SLAM_->GetGraphSnapshot().active_map_id;
    }
    feature_stats_.push_back(stat);
}

namespace
{
// Pose columns: camera position in the world frame and orientation as an xyzw
// quaternion (T_world_camera).
void write_pose(std::ostream& out, const Sophus::SE3f& T_world_camera)
{
    const Eigen::Vector3f& t = T_world_camera.translation();
    const Eigen::Quaternionf& q = T_world_camera.unit_quaternion();
    out << t.x() << "," << t.y() << "," << t.z() << ","
        << q.x() << "," << q.y() << "," << q.z() << "," << q.w();
}
}  // namespace

size_t MonocularSLAMNode::save_feature_stats(const std::string& csv_path) const
{
    std::ofstream out(csv_path);
    if(!out)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not write feature stats to '%s'", csv_path.c_str());
        return 0;
    }

    out << "stamp,detected,matched,state,map_id,x,y,z,qx,qy,qz,qw\n";
    out << std::fixed << std::setprecision(6);
    for(const FeatureStat& s : feature_stats_)
    {
        out << s.stamp << "," << s.detected << "," << s.matched << "," << s.state << ",";
        if(s.tracked)
        {
            out << s.map_id << ",";
            write_pose(out, s.T_world_camera);
        }
        else
            out << ",,,,,,,";
        out << "\n";
    }

    RCLCPP_INFO(this->get_logger(), "Saved feature stats for %zu frames to '%s'",
                feature_stats_.size(), csv_path.c_str());
    return feature_stats_.size();
}

void MonocularSLAMNode::save_map(const std::string& stem) const
{
    std::ofstream kf_out(stem + "_keyframes.csv");
    std::ofstream mp_out(stem + "_map_points.csv");
    if(!kf_out || !mp_out)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not write map files with prefix '%s'", stem.c_str());
        return;
    }

    size_t num_keyframes = 0;
    kf_out << "map_id,id,x,y,z,qx,qy,qz,qw\n" << std::fixed << std::setprecision(6);
    for(const ORB_SLAM3::KeyframeSnapshot& kf : SLAM_->GetAllKeyframeSnapshots())
    {
        if(kf.bad)
            continue;
        kf_out << kf.map_id << "," << kf.id << ",";
        write_pose(kf_out, kf.T_world_camera);
        kf_out << "\n";
        num_keyframes++;
    }

    const std::vector<ORB_SLAM3::MapPointSnapshot> points = SLAM_->GetAllMapPointSnapshots();
    mp_out << "map_id,x,y,z\n" << std::fixed << std::setprecision(4);
    for(const ORB_SLAM3::MapPointSnapshot& p : points)
    {
        const Eigen::Vector3f& pos = p.position_world;
        mp_out << p.map_id << "," << pos.x() << "," << pos.y() << "," << pos.z() << "\n";
    }

    RCLCPP_INFO(this->get_logger(), "Saved %zu keyframes and %zu map points to '%s_*.csv'",
                num_keyframes, points.size(), stem.c_str());
}

void MonocularSLAMNode::grab_data(const ImageMsg::SharedPtr msg)
{
    try
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat gray_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

        if(is_glitchy_frame(gray_image))
            return;

        // Some bags (e.g. tello_alpha) carry zero header stamps. ORB_SLAM3 needs
        // increasing timestamps (lost-tracking timeouts), so fall back to the
        // receive time.
        double stamp = Utility::StampToSec(msg->header.stamp);
        if(stamp <= 0.0)
            stamp = this->get_clock()->now().seconds();

        current_pose_ = SLAM_->TrackMonocular(gray_image, stamp);
        record_feature_stats(stamp, current_pose_);

        PoseStampedMsg pose_msg1;
        pose_msg1.header.stamp = msg->header.stamp;
        pose_msg1.pose.position.x = current_pose_.translation()(0);
        pose_msg1.pose.position.y = current_pose_.translation()(1);
        pose_msg1.pose.position.z = current_pose_.translation()(2);

        Eigen::Quaternionf quaternion(current_pose_.rotationMatrix());
        pose_msg1.pose.orientation.x = -quaternion.x();
        pose_msg1.pose.orientation.y = -quaternion.y();
        pose_msg1.pose.orientation.z = -quaternion.z();
        pose_msg1.pose.orientation.w = quaternion.w();

        camera_pose_pub1_->publish(pose_msg1);

        // Skip publishing if tracking is not OK or pose is not set / NaN
        int tracking_state = SLAM_->GetTrackingState();
        if(tracking_state != ORB_SLAM3::Tracking::OK && tracking_state != ORB_SLAM3::Tracking::OK_KLT)
            return;

        Sophus::SE3f Twc = current_pose_.inverse();
        if(Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0,0)) // avoid publishing NaN
            return;
    
        rclcpp::Time msg_time = this->get_clock()->now();

        publish_odometry(Twc, msg_time);
        publish_camera_pose(Twc, msg->header);
        publish_tracked_points(SLAM_->GetTrackedMapPoints(), msg_time);
        publish_all_points(GetAllMapPoints(), msg_time);
        publish_tracking_img(cv_ptr->image, msg->header.stamp);
        publish_kf_markers(GetAllKeyframePoses(), msg_time);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}


void MonocularSLAMNode::publish_odometry(const Sophus::SE3f& Tcw_SE3f, const rclcpp::Time& msg_time)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg_time;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "camera";

    odom_msg.pose.pose.position.x = Tcw_SE3f.translation().x();
    odom_msg.pose.pose.position.y = Tcw_SE3f.translation().y();
    odom_msg.pose.pose.position.z = Tcw_SE3f.translation().z();

    Eigen::Quaternionf quaternion(Tcw_SE3f.rotationMatrix());
    odom_msg.pose.pose.orientation.x = quaternion.x();
    odom_msg.pose.pose.orientation.y = quaternion.y();
    odom_msg.pose.pose.orientation.z = quaternion.z();
    odom_msg.pose.pose.orientation.w = quaternion.w();

    odom_pub_->publish(odom_msg);
}

void MonocularSLAMNode::publish_tracking_img(cv::Mat image, const rclcpp::Time& msg_time)
{
    std_msgs::msg::Header header;
    
    header.stamp = msg_time;
    header.frame_id = "map";

    auto rendered_image_msg = std::make_unique<sensor_msgs::msg::Image>();

    rendered_image_msg->header = header;
    rendered_image_msg->height = image.rows;
    rendered_image_msg->width = image.cols;
    rendered_image_msg->encoding = "bgr8";
    rendered_image_msg->is_bigendian = 0;
    rendered_image_msg->step = image.step[0];
    rendered_image_msg->data.resize(image.total() * image.elemSize());
    std::memcpy(rendered_image_msg->data.data(), image.data, rendered_image_msg->data.size());

    tracking_img_pub_->publish(std::move(rendered_image_msg));
}

void MonocularSLAMNode::publish_kf_markers(const std::vector<Sophus::SE3f> kf_poses, const rclcpp::Time& msg_time)
{
    int numKFs = kf_poses.size();
    if (numKFs == 0)
        return;
    
    visualization_msgs::msg::Marker kf_markers;
    kf_markers.header.frame_id = "map";
    kf_markers.header.stamp = msg_time;
    kf_markers.ns = "kf_markers";
    kf_markers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    kf_markers.action = visualization_msgs::msg::Marker::ADD;
    kf_markers.pose.orientation.w = 1.0;
    kf_markers.lifetime = rclcpp::Duration::from_seconds(1.0);

    kf_markers.id = 0;
    kf_markers.scale.x = 0.05;
    kf_markers.scale.y = 0.05;
    kf_markers.scale.z = 0.05;
    kf_markers.color.g = 1.0;
    kf_markers.color.a = 1.0;

    for(size_t i = 0; i < numKFs; i++)
    {
        geometry_msgs::msg::Point kf_marker;
        kf_marker.x = kf_poses[i].translation().x();
        kf_marker.y = kf_poses[i].translation().y();
        kf_marker.z = kf_poses[i].translation().z();
        kf_markers.points.push_back(kf_marker);
    }
    
    kf_markers_pub_->publish(kf_markers);
}

void MonocularSLAMNode::publish_camera_pose(const Sophus::SE3f& Tcw_SE3f, const std_msgs::msg::Header& header)
{
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = this->get_clock()->now();
    transform_msg.header.frame_id = "map";
    transform_msg.child_frame_id = "camera";

    transform_msg.transform.translation.x = Tcw_SE3f.translation().x();
    transform_msg.transform.translation.y = Tcw_SE3f.translation().y();
    transform_msg.transform.translation.z = Tcw_SE3f.translation().z();

    Eigen::Quaternionf yaw_minus_90_deg(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY()));

    Eigen::Quaternionf corrected_rotation = yaw_minus_90_deg * Tcw_SE3f.unit_quaternion();

    transform_msg.transform.rotation.w = corrected_rotation.w();
    transform_msg.transform.rotation.x = corrected_rotation.x();
    transform_msg.transform.rotation.y = corrected_rotation.y();
    transform_msg.transform.rotation.z = corrected_rotation.z();

    transform_broadcaster_.sendTransform(transform_msg);

    PoseStampedMsg pose_msg2;
    pose_msg2.header.frame_id = "map";
    pose_msg2.header.stamp = transform_msg.header.stamp;

    pose_msg2.pose.position.x = transform_msg.transform.translation.x;
    pose_msg2.pose.position.y = transform_msg.transform.translation.y;
    pose_msg2.pose.position.z = transform_msg.transform.translation.z;

    pose_msg2.pose.orientation = transform_msg.transform.rotation;

    camera_pose_pub2_->publish(pose_msg2);
}

void MonocularSLAMNode::save_map_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    // The mainline ORB_SLAM3 does not expose runtime SaveMap(); the atlas is
    // saved to this file automatically during Shutdown().
    SLAM_->SetSaveAtlasFile("SavedMap.osa");
    RCLCPP_INFO(this->get_logger(), "Atlas will be saved to 'SavedMap.osa' on shutdown");
}

bool MonocularSLAMNode::save_traj_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                      std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    SLAM_->SaveKeyFrameTrajectoryTUM("SavedTrajectory.txt");
    RCLCPP_INFO(this->get_logger(), "Trajectory has been saved to 'SavedTrajectory.txt'");
    return true;
}

void MonocularSLAMNode::setup_services()
{
    save_map_service_ = this->create_service<std_srvs::srv::Empty>(
        "save_map", std::bind(&MonocularSLAMNode::save_map_srv, this, std::placeholders::_1, std::placeholders::_2));

    save_traj_service_ = this->create_service<std_srvs::srv::Empty>(
        "save_traj", std::bind(&MonocularSLAMNode::save_traj_srv, this, std::placeholders::_1, std::placeholders::_2));
}

void MonocularSLAMNode::publish_tracked_points(const std::vector<ORB_SLAM3::MapPoint*>& tracked_points, const rclcpp::Time& msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(tracked_points, msg_time);
    tracked_mappoints_pub_->publish(cloud);
}

void MonocularSLAMNode::publish_all_points(const std::vector<ORB_SLAM3::MapPoint*>& all_points, const rclcpp::Time& msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(all_points, msg_time);
    all_mappoints_pub_->publish(cloud);
}

sensor_msgs::msg::PointCloud2 MonocularSLAMNode::mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, const rclcpp::Time& msg_time)
{
    const int num_channels = 3;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = msg_time;
    cloud.header.frame_id = "map";
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    for(size_t i = 0; i < num_channels; ++i) {
        cloud.fields[i].name = (i == 0) ? "x" : (i == 1) ? "y" : "z";
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[i].count = 1;
    }

    cloud.data.resize(cloud.width * cloud.point_step);
    unsigned char* ptr = cloud.data.data();

    for(size_t i = 0; i < map_points.size(); ++i) {
        ORB_SLAM3::MapPoint* pMP = map_points[i];
        if(pMP)
        {
            float* p = reinterpret_cast<float*>(ptr);
            p[0] = pMP->GetWorldPos()(0);
            p[1] = pMP->GetWorldPos()(1);
            p[2] = pMP->GetWorldPos()(2);
            ptr += cloud.point_step;
        }
    }

    return cloud;
}

std::vector<ORB_SLAM3::MapPoint*> MonocularSLAMNode::GetAllMapPoints()
{
    ORB_SLAM3::GraphSnapshot snap = SLAM_->GetGraphSnapshot();
    // This version of ORB_SLAM3 only exposes the tracked points publicly,
    // so "all points" falls back to the current tracked set.
    return SLAM_->GetTrackedMapPoints();
}

std::vector<Sophus::SE3f> MonocularSLAMNode::GetAllKeyframePoses()
{
    std::vector<Sophus::SE3f> kf_poses;
    ORB_SLAM3::GraphSnapshot snap = SLAM_->GetGraphSnapshot();
    kf_poses.reserve(snap.keyframes.size());
    for(const ORB_SLAM3::KeyframeSnapshot& kf : snap.keyframes)
    {
        if(!kf.bad)
            kf_poses.push_back(kf.T_world_camera);
    }
    return kf_poses;
}
