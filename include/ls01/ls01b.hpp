//
// Created by user on 5/23/22.
//

#ifndef LS01B_WS_LS01B_HPP
#define LS01B_WS_LS01B_HPP

#include <ls01/serial.hpp>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#define LS01B_HEADER_LEN 6
#define LS01B_DATA_LEN 3
#define LS01B_PKT_ANGLE 15
#define LS01B_N_PKT (360 / LS01B_PKT_ANGLE)

namespace LS01 {
    class LS01B : public rclcpp::Node {
    public:
        explicit LS01B(const rclcpp::NodeOptions &options);

        ~LS01B() override;

    private:
        void validate_resolution() {
            if (resolution >= 1) {
                resolution_u8 = 100;
                resolution = 1;
                measurement_count = 360;
            } else if (resolution >= 0.5) {
                resolution_u8 = 50;
                resolution = 0.5;
                measurement_count = 360 * 2;
            } else {
                resolution_u8 = 25;
                resolution = 0.25;
                measurement_count = 360 * 4;
            }
        }

        void recv_thread();

        int config();

        int start() const;

        int stop() const;

        int serial_write(const void *buf, size_t n) const;

        int process_packet();

        void prepare_packet();

        void finish_packet();

        int serial_fd = -1;
        std::string lidar_frame;
        std::thread lidar_thread;
        float resolution;
        int speed_rpm;
        rclcpp::Time msg_time;
        rclcpp::Time last_pkt_time;
        rclcpp::Time last_config_time;
        uint8_t resolution_u8;
        int measurement_count;
        bool terminate_thread = false;
        static constexpr int timeout = 2000;
        int scan_per_packet;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
        sensor_msgs::msg::LaserScan::UniquePtr scan_msg;
        std::vector<uint8_t> pkt_buffer;
    };
}

#endif //LS01B_WS_LS01B_HPP
