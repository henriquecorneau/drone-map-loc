/*
 * Code from https://github.com/BojanAndonovski71/orb_slam3_ros2 and
 *           https://github.com/ozandmrz/orb_slam3_ros2_mono_publisher
 */

// System includes
#include <Eigen/Dense>

// OpenCV includes
#include <opencv2/core/core.hpp>

// Local includes
#include "rgbd-slam-node.hpp"

using std::placeholders::_1;

RGBDSLAMNode::RGBDSLAMNode(ORB_SLAM3::System* pSLAM)
:   Node("ORBSLAM3_ROS2"),
    transform_broadcaster_(this)
{
    SLAM_ = pSLAM;

    initialize_subscribers();

    // Camera pose publisher
    camera_pose_pub_ = this->create_publisher<PoseStampedMsg>(
        std::string(this->get_name()) + "/pose", rclcpp::QoS(10));

    // Tracked map points publisher
    tracked_mappoints_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        std::string(this->get_name()) + "/tracked_mappoints", rclcpp::QoS(10));

    // All map points publisher
    all_mappoints_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        std::string(this->get_name()) + "/all_mappoints", rclcpp::QoS(10));
    
    // Tracking image publisher
    tracking_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        std::string(this->get_name()) + "/tracking_image", rclcpp::QoS(10));
    
    // Keyframe markers publisher
    kf_markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        std::string(this->get_name()) + "/kf_markers", rclcpp::QoS(1000));
    
    // Odometry publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        std::string(this->get_name()) + "/odom", rclcpp::QoS(10));

    current_pose_ = Sophus::SE3f();
    
    setup_services();
}

void RGBDSLAMNode::initialize_subscribers()
{   
    // Image subscribers
    rgb_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/image_raw"); //cant use make_shared here> double free error
    depth_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/depth/image_raw");

    // Sync RGB and depth images
    sync_approximate_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub_, *depth_sub_);
    sync_approximate_->registerCallback(&RGBDSLAMNode::grab_data, this);
}

RGBDSLAMNode::~RGBDSLAMNode()
{
    SLAM_->Shutdown();
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RGBDSLAMNode::grab_data(const ImageMsg::SharedPtr msg_RGB, const ImageMsg::SharedPtr msg_D)
{
    try
    {
        cv_ptrRGB_ = cv_bridge::toCvShare(msg_RGB);
        cv_ptrD_ = cv_bridge::toCvShare(msg_D);

        current_pose_ = SLAM_->TrackRGBD(cv_ptrRGB_->image, cv_ptrD_->image, Utility::StampToSec(msg_RGB->header.stamp));

        Sophus::SE3f Twc = SLAM_->GetCamTwc();
        if(Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0,0)) // do not publish anything if the pose is NaN
            return;
    
        rclcpp::Time msg_time = this->get_clock()->now();
        publish_odometry(Twc, msg_time);
        publish_camera_pose(Twc, msg_time);
        publish_tracked_points(SLAM_->GetTrackedMapPoints(), msg_time);
        publish_all_points(SLAM_->GetAllMapPoints(), msg_time);
        publish_tracking_img(SLAM_->GetCurrentFrame(), msg_RGB->header.stamp);
        publish_kf_markers(SLAM_->GetAllKeyframePoses(), msg_time);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

void RGBDSLAMNode::publish_camera_pose(const Sophus::SE3f& Tcw_SE3f, const rclcpp::Time& msg_time)
{
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = msg_time;
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

    PoseStampedMsg pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = transform_msg.header.stamp;

    pose_msg.pose.position.x = transform_msg.transform.translation.x;
    pose_msg.pose.position.y = transform_msg.transform.translation.y;
    pose_msg.pose.position.z = transform_msg.transform.translation.z;

    pose_msg.pose.orientation = transform_msg.transform.rotation;

    camera_pose_pub_->publish(pose_msg);
}

void RGBDSLAMNode::publish_tracked_points(const std::vector<ORB_SLAM3::MapPoint*>& tracked_points, const rclcpp::Time& msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(tracked_points, msg_time);
    tracked_mappoints_pub_->publish(cloud);
}

void RGBDSLAMNode::publish_all_points(const std::vector<ORB_SLAM3::MapPoint*>& all_points, const rclcpp::Time& msg_time)
{
    sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(all_points, msg_time);
    all_mappoints_pub_->publish(cloud);
}

void RGBDSLAMNode::publish_tracking_img(cv::Mat image, const rclcpp::Time& msg_time)
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

void RGBDSLAMNode::publish_kf_markers(const std::vector<Sophus::SE3f> vKFposes, const rclcpp::Time& msg_time)
{
    int numKFs = vKFposes.size();
    if(numKFs == 0)
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

    for (int i = 0; i < numKFs; i++)
    {
        geometry_msgs::msg::Point kf_marker;
        kf_marker.x = vKFposes[i].translation().x();
        kf_marker.y = vKFposes[i].translation().y();
        kf_marker.z = vKFposes[i].translation().z();
        kf_markers.points.push_back(kf_marker);
    }
    
    kf_markers_pub_->publish(kf_markers);
}

void RGBDSLAMNode::publish_odometry(const Sophus::SE3f& Tcw_SE3f, const rclcpp::Time& msg_time)
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

sensor_msgs::msg::PointCloud2 RGBDSLAMNode::mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, const rclcpp::Time& msg_time)
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

    for (int i = 0; i < num_channels; ++i)
    {
        cloud.fields[i].name = (i == 0) ? "x" : (i == 1) ? "y" : "z";
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[i].count = 1;
    }

    cloud.data.resize(cloud.width * cloud.point_step);
    unsigned char* ptr = cloud.data.data();

    for (size_t i = 0; i < map_points.size(); ++i)
    {
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

void RGBDSLAMNode::setup_services()
{
    save_map_service_ = this->create_service<std_srvs::srv::Empty>(
        "save_map", std::bind(&RGBDSLAMNode::save_map_srv, this, std::placeholders::_1, std::placeholders::_2));

    save_trajectory_service_ = this->create_service<std_srvs::srv::Empty>(
        "save_traj", std::bind(&RGBDSLAMNode::save_trajectory_srv, this, std::placeholders::_1, std::placeholders::_2));
}

bool RGBDSLAMNode::save_map_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                      std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    SLAM_->SaveMap("SavedMap.map");
    RCLCPP_INFO(this->get_logger(), "Map has been saved to 'SavedMap.map'");
    return true;
}

bool RGBDSLAMNode::save_trajectory_srv(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                       std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    SLAM_->SaveKeyFrameTrajectoryTUM("SavedTrajectory.txt");
    RCLCPP_INFO(this->get_logger(), "Trajectory has been saved to 'SavedTrajectory.txt'");
    return true;
}