/*
 * Code from https://github.com/ozandmrz/orb_slam3_ros2_mono_publisher
 */

// System includes
#include <cstdlib>
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

// Parses map/localization options from argv, removes them from argv (so
// rclcpp::init does not reject them) and exports them as the
// environment variables read by this version of ORB_SLAM3
// (see ORB_SLAM3/src/System.cc).
std::vector<char*> ExtractAtlasArgs(int argc, char** argv, std::string& load_file,
                                    std::string& save_file, bool& localize_only,
                                    std::string& features_csv, bool& plot_features)
{
    std::vector<char*> remaining;
    remaining.push_back(argv[0]);

    for(int i = 1; i < argc; i++)
    {
        const std::string arg = argv[i];
        const bool has_next = (i + 1 < argc);

        if(arg == "--load-map" && has_next)
        {
            load_file = argv[++i];
        }
        else if(arg == "--save-map" && has_next)
        {
            save_file = argv[++i];
        }
        else if(arg == "--localize-only")
        {
            localize_only = true;
        }
        else if(arg == "--features-csv" && has_next)
        {
            features_csv = argv[++i];
        }
        else if(arg == "--no-plot")
        {
            plot_features = false;
        }
        else
        {
            remaining.push_back(argv[i]);
        }
    }
    return remaining;
}

int main(int argc, char **argv)
{
    std::string load_file, save_file;
    std::string features_csv = "/home/dev/maps/localization_features.csv";
    bool localize_only = false;
    bool plot_features = true;
    std::vector<char*> remaining_args = ExtractAtlasArgs(argc, argv, load_file, save_file, localize_only,
                                                         features_csv, plot_features);
    if(remaining_args.size() < 3)
    {
        std::cerr << "\nUsage: ros2 run ros2_orbslam3 mono path_to_vocabulary path_to_settings [--load-map map.osa] [--save-map map.osa] [--localize-only] [--features-csv stats.csv] [--no-plot]" << std::endl;
        return 1;
    }
    if(localize_only && load_file.empty())
    {
        std::cerr << "--localize-only requires --load-map <map.osa>" << std::endl;
        return 1;
    }

    if(!load_file.empty())
        setenv("ORB_SLAM3_LOAD_ATLAS", load_file.c_str(), 1);
    if(!save_file.empty())
        setenv("ORB_SLAM3_SAVE_ATLAS", save_file.c_str(), 1);
    if(localize_only)
        setenv("ORB_SLAM3_LOCALIZE_ONLY", "1", 1);

    rclcpp::init(static_cast<int>(remaining_args.size()), remaining_args.data());

    bool visualization = true;
    ORB_SLAM3::System SLAM(remaining_args[1], remaining_args[2], ORB_SLAM3::System::MONOCULAR, visualization);

    auto node = std::make_shared<MonocularSLAMNode>(&SLAM);
    std::cout << "============================ " << std::endl;
    rclcpp::spin(node);

    const size_t num_frames = node->save_feature_stats(features_csv);
    const std::string map_stem = features_csv.substr(0, features_csv.rfind(".csv"));
    node->save_map(map_stem);
    node.reset();  // shuts SLAM down (and saves the atlas) before plotting
    rclcpp::shutdown();

    if(plot_features && num_frames > 0)
    {
        // Both windows open at once; wait until the user closes them.
        const std::string csv = "'" + features_csv + "'";
        const std::string cmd = "ros2 run ros2_orbslam3 plot_features.py " + csv +
                                " & ros2 run ros2_orbslam3 plot_localization_map.py " + csv + " & wait";
        if(std::system(cmd.c_str()) != 0)
            std::cerr << "Plotting failed; run manually: " << cmd << std::endl;
    }

    return 0;
}
