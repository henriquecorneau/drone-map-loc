/*
 * Code from https://github.com/ozandmrz/orb_slam3_ros2_mono_publisher
 */

// System includes
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

// ORB SLAM3 includes
#include <System.h>

// Local includes
#include "monocular-slam-node.hpp"

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    bool visualization = true;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);

    auto node = std::make_shared<MonocularSLAMNode>(&SLAM);
    std::cout << "============================ " << std::endl;\

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
