// SPDX-License-Identifier: BSD-1-Clause
/*
 * ROS2 node for LeiShen LS01B v2 Lidar
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */
#pragma once

#include <ls01/ls01.hpp>

#define LS01B_HEADER_LEN 6
#define LS01B_DATA_LEN 3
#define LS01B_PKT_ANGLE 15
#define LS01B_N_PKT (360 / LS01B_PKT_ANGLE)

namespace LS01 {
    class LS01B : public LS01 {
    public:
        explicit LS01B(const rclcpp::NodeOptions &options);

        ~LS01B() override;

    private:
        void validate_resolution() {
            if (resolution >= 1) {
                resolution_u8 = 100;
                resolution = 1;
                measurement_count = 360;
                set_angle_multiplier(1);
            } else if (resolution >= 0.5) {
                resolution_u8 = 50;
                resolution = 0.5;
                measurement_count = 360 * 2;
                set_angle_multiplier(2);
            } else {
                resolution_u8 = 25;
                resolution = 0.25;
                measurement_count = 360 * 4;
                set_angle_multiplier(4);
            }
        }

        void recv_thread();

        int config();

        int start() const;

        int stop() const;

        int process_packet();

        void prepare_packet();

        void finish_packet();

        float resolution;
        int speed_rpm;
        rclcpp::Time msg_time;
        rclcpp::Time last_pkt_time;
        rclcpp::Time last_config_time;
        uint8_t resolution_u8;
        int measurement_count;
        int scan_per_packet;
        std::vector<uint8_t> pkt_buffer;
    };
}
