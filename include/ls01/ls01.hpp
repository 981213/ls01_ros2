// SPDX-License-Identifier: BSD-1-Clause
/*
 * Open a serial port on Linux
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */
#pragma once

#include <thread>
#include <mutex>
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
        int angle_multiplier = 1;
        std::string lidar_frame;
        std::thread lidar_thread;
        bool terminate_thread = false;
        static constexpr int timeout = 2000;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
        sensor_msgs::msg::LaserScan::UniquePtr scan_msg;
        std::vector<int16_t> scan_disable_bounds;
        std::mutex scan_bounds_lock;

        void filter_scan() {
            std::lock_guard<std::mutex> guard(scan_bounds_lock);
            for (size_t i = 0; i < scan_disable_bounds.size(); i += 2) {
                int start = scan_disable_bounds[i] * angle_multiplier;
                int count = (scan_disable_bounds[i + 1] - scan_disable_bounds[i]) * angle_multiplier;
                memset(&(scan_msg->ranges[start]), 0, count * sizeof(scan_msg->ranges[0]));
                memset(&(scan_msg->intensities[start]), 0, count * sizeof(scan_msg->intensities[0]));
            }
        }

        int serial_write(const void *buf, size_t n) const;

    private:
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle;

        void open_serial(const char *port, speed_t baud_option);

        void set_scan_bounds(std::vector<int16_t> bounds);
    };


}

