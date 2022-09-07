// SPDX-License-Identifier: BSD-1-Clause
/*
 * ROS2 node for LeiShen LSN10 v2 Lidar
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */
#pragma once

#include <ls01/ls01.hpp>

#define LSN10_PKT_LEN 58
#define LSN10_RESOL 80
#define LSN10_RESOL_RAD (0.8 / 360 * 2 * M_PI);
#define LSN10_MEASUREMENT_CNT (36000 / LSN10_RESOL)
#define LSN10_MEASUREMENT_PER_PKT 16

namespace LS01 {
    class LSN10 : public LS01 {
    public:
        explicit LSN10(const rclcpp::NodeOptions &options);

        ~LSN10() override;

    private:

        void recv_thread();

        int start() const;

        int stop() const;

        int process_packet();

        void prepare_packet();

        void finish_packet();

        rclcpp::Time msg_time;
        rclcpp::Time last_pkt_time;
        std::vector<uint8_t> pkt_buffer;
        int last_stop_angle = 0;
    };
}
