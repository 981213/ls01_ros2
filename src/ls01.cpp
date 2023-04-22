// SPDX-License-Identifier: BSD-1-Clause
/*
 * Open a serial port on Linux
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */

#include <ls01/ls01.hpp>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdexcept>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>

namespace LS01 {
    LS01::LS01(const std::string &node_name, const rclcpp::NodeOptions &options, speed_t baud_option) :
            rclcpp::Node(node_name, options) {
        auto scan_topic = declare_parameter<std::string>("scan_topic", "scan");
        lidar_frame = declare_parameter<std::string>("lidar_frame", "laser_link");
        auto port = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        range_min = declare_parameter("range_min", 0.1);
        range_max = declare_parameter("range_max", 8.0);
        scan_pub = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 1);
        declare_parameter("scan_disable_bounds", std::vector<long>{});
        param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
        auto cb = [this](const rclcpp::Parameter &p) {
            // types in ROS2 are messy...
            const auto &p_array = p.as_integer_array();
            std::vector<int16_t> p_i16;
            p_i16.reserve(p_array.size());
            for (const auto &i: p_array)
                p_i16.push_back((int16_t) i);
            this->set_scan_bounds(std::move(p_i16));
        };
        cb(get_parameter("scan_disable_bounds"));
        cb_handle = param_subscriber->add_parameter_callback("scan_disable_bounds", cb);
        open_serial(port.c_str(), baud_option);
    }

    LS01::~LS01() {
        close(serial_fd);
    }

    void LS01::open_serial(const char *port, speed_t baud_option) {
        int ret;
        struct termios tty;
        // Use O_NDELAY to ignore DCD state
        serial_fd = ::open(port, O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd < 0) {
            throw std::runtime_error(strerror(errno));
        }

        // Disallow other processes to open the port.
        if (ioctl(serial_fd, TIOCEXCL) < 0)
            throw std::runtime_error(strerror(errno));

        ret = fcntl(serial_fd, F_GETFL);
        if (ret == -1) {
            throw std::runtime_error(strerror(errno));
        }

        ret = fcntl(serial_fd, F_SETFL, ret | O_NONBLOCK);
        if (ret != 0) {
            throw std::runtime_error(strerror(errno));
        }

        if (tcgetattr(serial_fd, &tty) != 0) {
            throw std::runtime_error(strerror(errno));
        }

        cfsetospeed(&tty, baud_option);
        cfsetispeed(&tty, baud_option);

        tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        tty.c_cflag |= (CS8 | CLOCAL | CREAD);
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | IGNCR | INLCR);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;

        if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
            throw std::runtime_error(strerror(errno));
        }

        ret = tcflush(serial_fd, TCIOFLUSH);
        if (ret != 0) {
            throw std::runtime_error(strerror(errno));
        }
    }

    int LS01::serial_write(const void *buf, size_t n) const {
        auto ret = write(serial_fd, buf, n);
        if (ret < 0)
            return errno;
        else if ((size_t) ret != n)
            return -EINVAL;
        return 0;
    }

    void LS01::set_scan_bounds(std::vector<int16_t> bounds) {
        if (bounds.size() & 1)
            bounds.push_back(360);
        for (size_t i = 0; i < bounds.size(); i += 2) {
            if (bounds[i + 1] <= bounds[i]) {
                RCLCPP_WARN(get_logger(), "invalid scan disable bounds: upper bound %d is smaller than lower bound %d",
                            bounds[i + 1], bounds[i]);
                return;
            }
            bounds[i] *= angle_multiplier;
            bounds[i + 1] *= angle_multiplier;
        }
        std::lock_guard<std::mutex> guard(scan_bounds_lock);
        scan_disable_bounds = std::move(bounds);
    }

    void LS01::set_angle_multiplier(float multiplier) {
        std::lock_guard<std::mutex> guard(scan_bounds_lock);
        for (auto &i: scan_disable_bounds)
            i = i / angle_multiplier * multiplier;
        angle_multiplier = multiplier;
    }
}