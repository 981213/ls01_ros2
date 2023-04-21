// SPDX-License-Identifier: BSD-1-Clause
/*
 * ROS2 node for LeiShen LSN10 v2 Lidar
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */
#pragma once

#include <ls01/ls01.hpp>

#define LSN10_PKT_LEN 58
#define LSN10P_PKT_LEN 108
#define LSN10_MEASUREMENT_PER_PKT 16
#define LSN10P_MEASUREMENT_PER_PKT 32
#define LSN10_ANGLE_NUM_SAMPLES 200
#define LSN10_ANGLE_STABLE_COUNT 5

namespace LS01 {
    class LSN10Common : public LS01 {
    public:
        LSN10Common(const std::string &node_name, const rclcpp::NodeOptions &options, speed_t baud_option,
                    int _measurement_per_pkt)
                : LS01(node_name, options, baud_option), measurement_per_pkt(_measurement_per_pkt) {}

        ~LSN10Common() override;

    protected:
        void recv_thread();

        std::vector<uint8_t> pkt_buffer;
    private:
        int start() const;

        int stop() const;

        int process_packet();

        int probe_params();

        void prepare_packet();

        void finish_packet();

        rclcpp::Time msg_time;
        rclcpp::Time last_pkt_time;
        uint32_t last_stop_angle = 0;
        int measurement_per_pkt;

        // lidar param probing
        uint32_t angle_increment = 0;
        uint32_t angle_stable_count = 0;
        uint32_t measurement_cnt;
        float angle_incr_rad = 0;
        float angle_incr_acc = 0;
        uint32_t angle_incr_acc_cnt = 0;
        bool lidar_params_determined = false;

    };

    class LSN10 : public LSN10Common {
    public:
        explicit LSN10(const rclcpp::NodeOptions &options)
                : LSN10Common("lsn10", options, B230400, LSN10_MEASUREMENT_PER_PKT) {
            pkt_buffer.resize(LSN10_PKT_LEN);
            lidar_thread = std::thread(&LSN10::recv_thread, this);
        }
    };

    class LSN10P : public LSN10Common {
    public:
        explicit LSN10P(const rclcpp::NodeOptions &options)
                : LSN10Common("lsn10p", options, B460800, LSN10P_MEASUREMENT_PER_PKT) {
            pkt_buffer.resize(LSN10P_PKT_LEN);
            lidar_thread = std::thread(&LSN10P::recv_thread, this);
        }
    };
}
