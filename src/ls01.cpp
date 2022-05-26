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
        scan_pub = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 1);
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
}