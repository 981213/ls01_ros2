// SPDX-License-Identifier: BSD-1-Clause
/*
 * Standalone ROS2 executable for LeiShen LS01B
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */

#include <ls01/ls01b.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<LS01::LS01B>(options));
    rclcpp::shutdown();
    return 0;
}