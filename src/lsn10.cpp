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
    LSN10Common::~LSN10Common() {
        terminate_thread = true;
        lidar_thread.join();
        stop();
    }

    void LSN10Common::recv_thread() {
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
                        if (lidar_params_determined)
                            ret = process_packet();
                        else
                            ret = probe_params();
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

    int LSN10Common::start() const {
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

    int LSN10Common::stop() const {
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

    void LSN10Common::prepare_packet() {
        msg_time = last_pkt_time;
        scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        scan_msg->header.stamp = msg_time;
        scan_msg->header.frame_id = lidar_frame;
        scan_msg->angle_min = 0;
        scan_msg->angle_increment = angle_incr_rad;
        scan_msg->angle_max = 2 * M_PI - angle_incr_rad;
        scan_msg->range_min = range_min;
        scan_msg->range_max = range_max;
        scan_msg->intensities.resize(measurement_cnt, 0);
        scan_msg->ranges.resize(measurement_cnt, 0);
    }

    void LSN10Common::finish_packet() {
        scan_msg->scan_time = (last_pkt_time - msg_time).seconds();
        scan_msg->time_increment = scan_msg->scan_time / (float) measurement_cnt;
        filter_scan();
        scan_pub->publish(std::move(scan_msg));
    }

    int LSN10Common::probe_params() {
        if (pkt_buffer[0] != 0xa5 || pkt_buffer[1] != 0x5a)
            return -EINVAL;

        uint32_t start_angle = pkt_buffer[5] << 8 | pkt_buffer[6];
        uint32_t stop_angle = pkt_buffer[pkt_buffer.size() - 3] << 8 | pkt_buffer[pkt_buffer.size() - 2];
        uint32_t angle_diff = (stop_angle < start_angle ? stop_angle + 36000 : stop_angle) - start_angle;

        angle_incr_acc += angle_diff;
        angle_incr_acc_cnt++;

        if (angle_incr_acc_cnt < LSN10_ANGLE_NUM_SAMPLES)
            return 0;

        uint32_t cur_angle_increment = round(angle_incr_acc / LSN10_ANGLE_NUM_SAMPLES / (measurement_per_pkt - 1));
        angle_incr_acc = 0;
        angle_incr_acc_cnt = 0;

        if (cur_angle_increment != angle_increment) {
            angle_increment = cur_angle_increment;
            angle_stable_count = 0;
            RCLCPP_DEBUG(get_logger(), "Motor speed unstable. Current resolution: %.2f degree.",
                         angle_increment / 100.0);
        } else {
            angle_stable_count++;
        }

        if (angle_stable_count > LSN10_ANGLE_STABLE_COUNT) {
            lidar_params_determined = true;
            angle_increment = cur_angle_increment;
            float angle_incr_degree = cur_angle_increment / 100.0f;
            angle_incr_rad = angle_incr_degree / 360 * 2 * M_PI;
            measurement_cnt = 36000 / angle_increment;
            RCLCPP_INFO(get_logger(), "Motor stabilized. Resolution: %.2f degree.", angle_incr_degree);
            set_angle_multiplier(1.0f / angle_incr_degree);
            prepare_packet();
        }
        return 0;
    }

    int LSN10Common::process_packet() {
        if (pkt_buffer[0] != 0xa5 || pkt_buffer[1] != 0x5a)
            return -EINVAL;
        last_pkt_time = get_clock()->now();
        uint32_t start_angle = pkt_buffer[5] << 8 | pkt_buffer[6];
        uint32_t scan_ptr = (start_angle + angle_increment / 2) / angle_increment;
        if (start_angle < last_stop_angle) {
            finish_packet();
            prepare_packet();
        }
        last_stop_angle = pkt_buffer[pkt_buffer.size() - 3] << 8 | pkt_buffer[pkt_buffer.size() - 2];

        auto data_ptr = &pkt_buffer[7];
        for (int i = 0; i < measurement_per_pkt; i++) {
            if (scan_ptr >= measurement_cnt) {
                scan_ptr = 0;
                finish_packet();
                prepare_packet();
                // make sure we don't publish empty scans if this is the last measurement.
                last_stop_angle = 0;
            }
            // the scan data angle is clockwise while LaserScan message expects ccw. Invert it.
            uint32_t idx = measurement_cnt - scan_ptr - 1;
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
RCLCPP_COMPONENTS_REGISTER_NODE(LS01::LSN10P)
