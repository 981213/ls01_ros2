// SPDX-License-Identifier: BSD-1-Clause
/*
 * ROS2 node for LeiShen LSN10 v2 Lidar
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */
#include <ls01/lsn10.hpp>
#include <string>
#include <chrono>
#include <poll.h>
#include <unistd.h>

namespace LS01 {
    LSN10::LSN10(const rclcpp::NodeOptions &options) : LS01("lsn10", options, B230400) {
        pkt_buffer.resize(LSN10_PKT_LEN);
        // FIXME: angle_multiplier = 
        lidar_thread = std::thread(&LSN10::recv_thread, this);
    }

    LSN10::~LSN10() {
        terminate_thread = true;
        lidar_thread.join();
        stop();
    }

    void LSN10::recv_thread() {
        size_t buf_ptr = 0;
        ssize_t rdlen;
        enum class _sync_t {
            SYNC1,
            SYNC2,
            SYNC_SUCCESS,
        } sync_state;
        int ret;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        start();

        while (!terminate_thread) {
            pollfd pfd = {serial_fd, POLLIN, 0};
            ret = poll(&pfd, 1, timeout);
            if (ret == 0) {
                RCLCPP_WARN(get_logger(), "Lidar timeout. Restarting...");
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
                    if (pkt_buffer[1] == 0x5a) {
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
                        if (ret) {
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

    int LSN10::start() const {
        static const uint8_t start_motor[] = {0xa5, 0x5a, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0xfa, 0xfb};
        return serial_write(start_motor, sizeof(start_motor));
    }

    int LSN10::stop() const {
        static const uint8_t stop_motor[] = {0xa5, 0x5a, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xfa, 0xfb};
        return serial_write(stop_motor, sizeof(stop_motor));
    }

    void LSN10::prepare_packet() {
        msg_time = last_pkt_time;
        scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        scan_msg->header.stamp = msg_time;
        scan_msg->header.frame_id = lidar_frame;
        scan_msg->angle_min = 0;
        scan_msg->angle_increment = LSN10_RESOL_RAD;
        scan_msg->angle_max = 2 * M_PI - LSN10_RESOL_RAD;
        scan_msg->range_min = 0.1;
        scan_msg->range_max = 12.0;
        scan_msg->intensities.resize(LSN10_MEASUREMENT_CNT, 0);
        scan_msg->ranges.resize(LSN10_MEASUREMENT_CNT, 0);
    }

    void LSN10::finish_packet() {
        scan_msg->scan_time = (last_pkt_time - msg_time).seconds();
        scan_msg->time_increment = scan_msg->scan_time / (float) LSN10_MEASUREMENT_CNT;
        // FIXME: filter_scan();
        scan_pub->publish(std::move(scan_msg));
    }

    int LSN10::process_packet() {
        if (pkt_buffer[0] != 0xa5 || pkt_buffer[1] != 0x5a)
            return -EINVAL;
        last_pkt_time = get_clock()->now();
        int start_angle = pkt_buffer[5] << 8 | pkt_buffer[6];
        int scan_ptr = (start_angle + LSN10_RESOL / 2) / LSN10_RESOL;
        if (start_angle < last_stop_angle) {
            finish_packet();
            prepare_packet();
        }
        last_stop_angle = pkt_buffer[55] << 8 | pkt_buffer[56];
        auto data_ptr = &pkt_buffer[7];
        for (int i = 0; i < LSN10_MEASUREMENT_PER_PKT; i++) {
            if (scan_ptr >= LSN10_MEASUREMENT_CNT) {
                scan_ptr = 0;
                finish_packet();
                prepare_packet();
            }
            // the scan data angle is clockwise while LaserScan message expects ccw. Invert it.
            int idx = LSN10_MEASUREMENT_CNT - scan_ptr - 1;
            scan_msg->ranges[idx] = ((data_ptr[0] << 8) | data_ptr[1]) / 1000.0f;
            scan_msg->intensities[idx] = data_ptr[2];
            scan_ptr++;
            data_ptr += 3;
        }
        return 0;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(LS01::LSN10)
