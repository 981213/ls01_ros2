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
#define LSN10_ANGLE_NUM_SAMPLES 200
#define LSN10_ANGLE_STABLE_COUNT 5

namespace LS01 {
    class LSN10Common : public LS01 {
    public:
        LSN10Common(const std::string &node_name, const rclcpp::NodeOptions &options, speed_t baud_option)
                : LS01(node_name, options, baud_option) {
            scan_rate = declare_parameter("scan_rate", 10);
        }

        ~LSN10Common() override;

    protected:
        void recv_thread();

        std::vector<uint8_t> pkt_buffer;
    private:
        int start() const;

        int set_rate(uint8_t hz) const;

        int stop() const;

        int process_packet();

        virtual void extract_measurements(uint8_t *data, uint16_t *range, uint8_t *intensity) = 0;

        int probe_params();

        void prepare_packet();

        void finish_packet();

        rclcpp::Time msg_time;
        rclcpp::Time last_pkt_time;
        uint32_t last_stop_angle = 0;

        // lidar param probing
        uint32_t angle_increment = 0;
        uint32_t angle_stable_count = 0;
        uint32_t measurement_cnt;
        float angle_incr_rad = 0;
        double angle_incr_acc = 0;
        uint32_t angle_incr_acc_cnt = 0;
        bool lidar_params_determined = false;
        uint8_t scan_rate;

    };

    class LSN10 final : public LSN10Common {
    public:
        explicit LSN10(const rclcpp::NodeOptions &options)
                : LSN10Common("lsn10", options, B230400) {
            pkt_buffer.resize(LSN10_PKT_LEN);
            lidar_thread = std::thread(&LSN10::recv_thread, this);
        }

        void extract_measurements(uint8_t *data, uint16_t *ranges, uint8_t *intensities) override {
            for (int i = 0; i < LSN10_MEASUREMENT_PER_PKT; i++) {
                ranges[i] = ((data[0] << 8) | data[1]);
                intensities[i] = data[2];
                data+=3;
            }
        }

        ~LSN10() override {
            terminate_thread = true;
            lidar_thread.join();
        };
    };

    class LSN10P final : public LSN10Common {
    public:
        explicit LSN10P(const rclcpp::NodeOptions &options)
                : LSN10Common("lsn10p", options, B460800) {
            pkt_buffer.resize(LSN10P_PKT_LEN);
            lidar_thread = std::thread(&LSN10P::recv_thread, this);
        }

        void extract_measurements(uint8_t *data, uint16_t *ranges, uint8_t *intensities) override {
            for (int i = 0; i < LSN10_MEASUREMENT_PER_PKT; i++) {
                // take the minimum range between the two
                uint16_t range1 = ((data[0] << 8) | data[1]);
                uint16_t range2 = ((data[3] << 8) | data[4]);
                if (range1 != 0 && range2 != 0 && range2 < range1) {
                    ranges[i] = range2;
                    intensities[i] = data[5];
                } else {
                    ranges[i] = range1;
                    intensities[i] = data[2];
                }
                data += 6;
            }
        }

        ~LSN10P() override {
            terminate_thread = true;
            lidar_thread.join();
        };
    };
}
