// SPDX-License-Identifier: BSD-1-Clause
/*
 * ROS2 node for LeiShen LS01D Lidar
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */
#pragma once
#include <ls01/ls01.hpp>

#define LS01D_PKT_LEN 1812

#define LS01D_PKT_HEAD 0xa5
#define LS01D_PKT_B6 0x81
#define LS01D_PKT_END 0xdd
#define LS01D_PKT_HDR_LEN 7
#define LS01D_PKT_SCAN_LEN 5

#define LS01D_NUM_SCAN 360

#define LS01D_CMD(_cmd) {0xa5, _cmd, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd}
#define LS01D_CMD_LEN 7

namespace LS01 {
    class LS01D : public LS01 {
    public:
        explicit LS01D(const rclcpp::NodeOptions &options);

        ~LS01D() override;

    private:
        void recv_thread();

        int start() const;

        int stop() const;

        void process_scan(int start, int end);

        void prepare_packet();

        int finish_packet();

        rclcpp::Time last_pkt_time;
        rclcpp::Time msg_time;
        int scan_processed;
        // process the partial packet when we received this amount of scans
        static constexpr int process_threshold = 320;
        sensor_msgs::msg::LaserScan::UniquePtr scan_msg;
        uint8_t pkt_buffer[LS01D_PKT_LEN];
    };
}