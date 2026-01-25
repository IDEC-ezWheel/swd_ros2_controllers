/**
 * Copyright (C) 2022 ez-Wheel S.A.S.
 *
 * @file main.cpp
 */

#include <cstdio>
#include <rclcpp/rclcpp.hpp>

#include "diff_drive_controller/DiffDriveController.hpp"

auto main(int argc, char** argv) -> int
{
    // Force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create node
    auto node = std::make_shared<ezw::swd::DiffDriveController>("diff_drive_controller");

    // Start processing data from the node as well as the callbacks and the timer
    rclcpp::spin(node);

    // Shutdown the node when finished
    rclcpp::shutdown();

    return 0;
}
