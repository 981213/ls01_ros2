// SPDX-License-Identifier: BSD-1-Clause
/*
 * Standalone ROS2 executable for LeiShen LS01D
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */

#include <ls01/lsn10.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<LS01::LSN10>(options));
    rclcpp::shutdown();
    return 0;
}