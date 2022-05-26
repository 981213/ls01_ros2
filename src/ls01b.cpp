// SPDX-License-Identifier: BSD-1-Clause
/*
 * ROS2 node for LeiShen LS01B v2 Lidar
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */
#include <ls01/ls01b.hpp>
#include <string>
#include <chrono>
#include <poll.h>
#include <unistd.h>

namespace LS01 {
    LS01B::LS01B(const rclcpp::NodeOptions &options) : LS01("ls01b", options, B460800) {
        speed_rpm = declare_parameter<int>("speed_rpm", 600);
        resolution = declare_parameter<float>("resolution", 0.25);
        validate_resolution();
        scan_per_packet = LS01B_PKT_ANGLE * 100 / resolution_u8;
        pkt_buffer.resize(LS01B_HEADER_LEN + LS01B_DATA_LEN * scan_per_packet);
        lidar_thread = std::thread(&LS01B::recv_thread, this);
    }

    LS01B::~LS01B() {
        terminate_thread = true;
        lidar_thread.join();
        stop();
    }

    void LS01B::recv_thread() {
        size_t buf_ptr = 0;
        ssize_t rdlen;
        enum class _sync_t {
            SYNC1,
            SYNC2,
            SYNC_SUCCESS,
        } sync_state;
        int ret;

        config();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        start();

        while (!terminate_thread) {
            pollfd pfd = {serial_fd, POLLIN, 0};
            ret = poll(&pfd, 1, timeout);
            if (ret == 0) {
                RCLCPP_WARN(get_logger(), "Lidar timeout. Restarting...");
                config();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                start();
                continue;
            } else if (ret < 0) {
                RCLCPP_ERROR(get_logger(), "poll error: %s", strerror(errno));
                continue;
            }
            switch (sync_state) {
                case _sync_t::SYNC1:
                    rdlen = read(serial_fd, pkt_buffer.data(), 1);
                    if (rdlen != 1) {
                        RCLCPP_ERROR(get_logger(), "read error: %s", strerror(errno));
                        break;
                    }
                    if (pkt_buffer[0] == 0xa5) {
                        sync_state = _sync_t::SYNC2;
                        last_pkt_time = get_clock()->now();
                        prepare_packet();
                    }
                    break;
                case _sync_t::SYNC2:
                    rdlen = read(serial_fd, pkt_buffer.data() + 1, 1);
                    if (rdlen != 1) {
                        RCLCPP_ERROR(get_logger(), "read error: %s", strerror(errno));
                        sync_state = _sync_t::SYNC1;
                        break;
                    }
                    if (pkt_buffer[1] == 0x5a || pkt_buffer[1] == 0x6a) {
                        sync_state = _sync_t::SYNC_SUCCESS;
                        buf_ptr = 2;
                        RCLCPP_DEBUG(get_logger(), "packet header found.");
                    } else {
                        sync_state = _sync_t::SYNC1;
                    }
                    break;
                case _sync_t::SYNC_SUCCESS:
                    size_t pkt_left = pkt_buffer.size() - buf_ptr;
                    rdlen = read(serial_fd, pkt_buffer.data() + buf_ptr, pkt_left);
                    if (rdlen < 0) {
                        RCLCPP_ERROR(get_logger(), "read error: %s", strerror(errno));
                        sync_state = _sync_t::SYNC1;
                        break;
                    } else if ((size_t) rdlen == pkt_left) {
                        buf_ptr = 0;
                        ret = process_packet();
                        if (ret == -EOPNOTSUPP) {
                            if ((get_clock()->now() - last_config_time).seconds() > 5) {
                                RCLCPP_WARN(get_logger(), "lidar config mismatch after 5s. Reconfiguring...");
                                config();
                            }
                            sync_state = _sync_t::SYNC1;
                        } else if (ret) {
                            RCLCPP_WARN(get_logger(), "invalid packet.");
                            sync_state = _sync_t::SYNC1;
                        }
                    } else {
                        buf_ptr += rdlen;
                    }
                    break;
            }
        }
    }

    int LS01B::config() {
        int ret;
        uint8_t cmd_buf[4];
        cmd_buf[0] = 0xa5;
        // resolution
        cmd_buf[1] = 0x30;
        cmd_buf[2] = 0x00;
        cmd_buf[3] = resolution_u8;
        ret = serial_write(cmd_buf, sizeof(cmd_buf));
        if (ret)
            return ret;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // speed
        cmd_buf[1] = 0x26;
        cmd_buf[2] = speed_rpm >> 8;
        cmd_buf[3] = speed_rpm & 0xff;
        ret = serial_write(cmd_buf, sizeof(cmd_buf));
        if (ret)
            return ret;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // return intensity instead of angle
        cmd_buf[1] = 0x50;
        last_config_time = get_clock()->now();
        return serial_write(cmd_buf, 2);
    }

    int LS01B::start() const {
        int ret;
        const uint8_t start_motor[] = {0xa5, 0x2c};
        ret = serial_write(start_motor, sizeof(start_motor));
        if (ret)
            return ret;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const uint8_t cont_data_stream[] = {0xa5, 0x20};
        return serial_write(cont_data_stream, sizeof(cont_data_stream));
    }

    int LS01B::stop() const {
        int ret;
        const uint8_t stop_datastream[] = {0xa5, 0x21};
        ret = serial_write(stop_datastream, sizeof(stop_datastream));
        if (ret)
            return ret;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const uint8_t stop_motor[] = {0xa5, 0x25};
        return serial_write(stop_motor, sizeof(stop_motor));
    }

    void LS01B::prepare_packet() {
        msg_time = last_pkt_time;
        scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        scan_msg->header.stamp = msg_time;
        scan_msg->header.frame_id = lidar_frame;
        scan_msg->angle_min = 0;
        scan_msg->angle_increment = resolution / 360 * 2 * M_PI;
        scan_msg->angle_max = 2 * M_PI - scan_msg->angle_increment;
        scan_msg->range_min = 0.15;
        scan_msg->range_max = 16.0;
        scan_msg->intensities.resize(measurement_count, 0);
        scan_msg->ranges.resize(measurement_count, 0);
    }

    void LS01B::finish_packet() {
        scan_msg->scan_time = (last_pkt_time - msg_time).seconds();
        scan_msg->time_increment = scan_msg->scan_time / (float) measurement_count;
        filter_scan();
        scan_pub->publish(std::move(scan_msg));
    }

    int LS01B::process_packet() {
        if (pkt_buffer[0] != 0xa5)
            return -EINVAL;
        if (pkt_buffer[1] == 0x6a) {
            prepare_packet();
        } else if (pkt_buffer[1] != 0x5a) {
            return -EINVAL;
        }
        last_pkt_time = get_clock()->now();
        // check if lidar returns intensity. Return if not.
        if (!(pkt_buffer[2] & 0x80))
            return -EOPNOTSUPP;
        // Speed in RPM (unused): ((pkt_buffer[2] & 0x7f) << 8) | pkt_buffer[3];
        // check angular resolution
        if ((pkt_buffer[4] >> 1) != resolution_u8)
            return -EOPNOTSUPP;
        float data_angle = ((pkt_buffer[4] & 1) << 8) | pkt_buffer[5];

        auto data_ptr = &pkt_buffer[6];
        for (int i = 0; i < scan_per_packet; i++) {
            float dist = ((data_ptr[1] << 8) | data_ptr[2]) / 1000.0f;
            // angle compensation
            float angle_offset = (0.02345 / dist) * 57.3;
            float angle = data_angle - angle_offset;
            ssize_t idx = roundf(angle / resolution);
            // the scan data angle is clockwise while LaserScan message expects ccw. Invert it.
            if (idx < 0)
                idx = -idx;
            else
                idx = measurement_count - idx;
            scan_msg->intensities[idx] = data_ptr[0];
            scan_msg->ranges[idx] = dist;
            data_angle += resolution;
            data_ptr += 3;
        }

        if (data_angle >= 359.0f)
            finish_packet();
        return 0;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(LS01::LS01B)
