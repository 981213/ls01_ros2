// SPDX-License-Identifier: BSD-1-Clause
/*
 * ROS2 node for LeiShen LS01D Lidar
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */
#include <ls01/ls01d.hpp>
#include <string>
#include <chrono>
#include <poll.h>
#include <unistd.h>

namespace LS01 {
    LS01D::LS01D(const rclcpp::NodeOptions &options) : LS01("ls01d", options, B230400) {
        lidar_thread = std::thread(&LS01D::recv_thread, this);
    }

    LS01D::~LS01D() {
        terminate_thread = true;
        lidar_thread.join();
        stop();
    }

    void LS01D::recv_thread() {
        size_t buf_ptr = 0;
        size_t pkt_left;
        ssize_t rdlen;
        enum class _sync_t {
            HEADER,
            B6,
            SYNC_SUCCESS,
        } sync_state;
        int ret;

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
                case _sync_t::HEADER:
                    rdlen = read(serial_fd, pkt_buffer, 1);
                    if (rdlen != 1) {
                        RCLCPP_ERROR(get_logger(), "read error: %s", strerror(errno));
                        break;
                    }
                    if (pkt_buffer[0] == LS01D_PKT_HEAD) {
                        sync_state = _sync_t::B6;
                        last_pkt_time = get_clock()->now();
                        buf_ptr = 1;
                    }
                    break;
                case _sync_t::B6:
                    pkt_left = 7 - buf_ptr;
                    rdlen = read(serial_fd, pkt_buffer + buf_ptr, pkt_left);
                    if (rdlen < 0) {
                        RCLCPP_ERROR(get_logger(), "read error: %s", strerror(errno));
                        sync_state = _sync_t::HEADER;
                        break;
                    }
                    if ((size_t) rdlen == pkt_left) {
                        if (pkt_buffer[6] == LS01D_PKT_B6) {
                            sync_state = _sync_t::SYNC_SUCCESS;
                            RCLCPP_DEBUG(get_logger(), "packet header found.");
                            prepare_packet();
                            scan_processed = 0;
                        } else {
                            sync_state = _sync_t::HEADER;
                        }
                    }
                    buf_ptr += rdlen;
                    break;
                case _sync_t::SYNC_SUCCESS:
                    pkt_left = sizeof(pkt_buffer) - buf_ptr;
                    rdlen = read(serial_fd, pkt_buffer + buf_ptr, pkt_left);
                    if (rdlen < 0) {
                        RCLCPP_ERROR(get_logger(), "read error: %s", strerror(errno));
                        sync_state = _sync_t::HEADER;
                        break;
                    }
                    buf_ptr += rdlen;
                    if ((size_t) rdlen == pkt_left) {
                        last_pkt_time = get_clock()->now();
                        buf_ptr = 0;
                        process_scan(scan_processed, LS01D_NUM_SCAN);
                        ret = finish_packet();
                        if (ret) {
                            RCLCPP_WARN(get_logger(), "invalid packet.");
                            sync_state = _sync_t::HEADER;
                        }
                        prepare_packet();
                        scan_processed = 0;
                        // LS01D runs at 10rps -> 100 ms per scan. Wait 75ms for 3/4 packet.
                        std::this_thread::sleep_for(std::chrono::milliseconds(75));
                    } else if (!scan_processed &&
                               buf_ptr >= LS01D_PKT_HDR_LEN + process_threshold * LS01D_PKT_SCAN_LEN) {
                        process_scan(0, process_threshold);
                        scan_processed = process_threshold;
                    }
                    break;
            }
        }
    }

    void LS01D::prepare_packet() {
        msg_time = last_pkt_time;
        scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        scan_msg->header.stamp = last_pkt_time;
        scan_msg->header.frame_id = lidar_frame;
        scan_msg->angle_min = 0;
        scan_msg->angle_increment = 1.0 / 360 * 2 * M_PI;
        scan_msg->angle_max = 359.0 / 360 * 2 * M_PI;;
        scan_msg->range_min = 0.1;
        scan_msg->range_max = 10.0;
        scan_msg->intensities.resize(LS01D_NUM_SCAN, 0);
        scan_msg->ranges.resize(LS01D_NUM_SCAN, 0);
    }

    int LS01D::finish_packet() {
        if (pkt_buffer[0] != LS01D_PKT_HEAD || pkt_buffer[6] != LS01D_PKT_B6 ||
            pkt_buffer[LS01D_PKT_LEN - 1] != LS01D_PKT_END)
            return -EINVAL;
        // Unused lidar motor speed (rps): pkt_buffer[1] / 15.0;
        scan_msg->scan_time = (last_pkt_time - msg_time).seconds();
        scan_msg->time_increment = scan_msg->scan_time / (float) LS01D_NUM_SCAN;
        filter_scan();
        scan_pub->publish(std::move(scan_msg));
        return 0;
    }

    static inline uint16_t get_le16(const void *ptr) {
        return le16toh(*reinterpret_cast<const uint16_t *>(ptr));
    }

    void LS01D::process_scan(int start, int end) {
        auto data_ptr = &pkt_buffer[LS01D_PKT_HDR_LEN + start * LS01D_PKT_SCAN_LEN];
        for (int i = start; i < end; i++) {
            // LaserScan expects ccw data.
            int msg_ptr = LS01D_NUM_SCAN - i - 1;
            scan_msg->intensities[msg_ptr] = get_le16(data_ptr + 1);
            scan_msg->ranges[msg_ptr] = get_le16(data_ptr + 3) / 1000.0f;
            data_ptr += LS01D_PKT_SCAN_LEN;
        }
    }

    int LS01D::start() const {
        int ret;
        const uint8_t cmd_unk0[] = LS01D_CMD(0x3a);
        ret = serial_write(cmd_unk0, LS01D_CMD_LEN);
        if (ret)
            return ret;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        const uint8_t cmd_start[] = LS01D_CMD(0x2c);
        ret = serial_write(cmd_start, LS01D_CMD_LEN);
        if (ret)
            return ret;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        const uint8_t cmd_continuous_data[] = LS01D_CMD(0x20);
        ret = serial_write(cmd_continuous_data, LS01D_CMD_LEN);
        if (ret)
            return ret;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        const uint8_t cmd_intensity[] = LS01D_CMD(0x50);
        return serial_write(cmd_intensity, LS01D_CMD_LEN);
    }

    int LS01D::stop() const {
        int ret;
        const uint8_t cmd_stop_data[] = LS01D_CMD(0x21);
        ret = serial_write(cmd_stop_data, LS01D_CMD_LEN);
        if (ret)
            return ret;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        const uint8_t cmd_stop[] = LS01D_CMD(0x25);
        return serial_write(cmd_stop, LS01D_CMD_LEN);
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(LS01::LS01D)