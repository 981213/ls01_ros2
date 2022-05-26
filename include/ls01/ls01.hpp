// SPDX-License-Identifier: BSD-1-Clause
/*
 * Open a serial port on Linux
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */
#pragma once

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <termios.h>

namespace LS01 {
    class LS01 : public rclcpp::Node {
    public:
        LS01(const std::string &node_name, const rclcpp::NodeOptions &options, speed_t baud_option);

        ~LS01() override;

    protected:
        int serial_fd = -1;
        std::string lidar_frame;
        std::thread lidar_thread;
        bool terminate_thread = false;
        static constexpr int timeout = 2000;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;

        int serial_write(const void *buf, size_t n) const;

    private:
        void open_serial(const char *port, speed_t baud_option);
    };


}

