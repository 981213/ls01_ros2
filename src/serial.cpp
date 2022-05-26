// SPDX-License-Identifier: BSD-1-Clause
/*
 * Open a serial port on Linux
 *
 * Copyright (c) 2022 Chuanhong Guo <gch981213@gmail.com>
 */

#include <ls01/serial.hpp>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdexcept>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>

int LS01::open_serial(const char *port, speed_t baud_option) {
    int ret;
    int serial_fd;
    struct termios tty;
    // Use O_NDELAY to ignore DCD state
    serial_fd = ::open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd < 0) {
        throw std::runtime_error(strerror(errno));
    }

    // Disallow other processes to open the port.
    if (ioctl(serial_fd, TIOCEXCL) < 0)
        throw std::runtime_error(strerror(errno));

    ret = fcntl(serial_fd, F_GETFL);
    if (ret == -1) {
        throw std::runtime_error(strerror(errno));
    }

    ret = fcntl(serial_fd, F_SETFL, ret | O_NONBLOCK);
    if (ret != 0) {
        throw std::runtime_error(strerror(errno));
    }

    if (tcgetattr(serial_fd, &tty) != 0) {
        throw std::runtime_error(strerror(errno));
    }

    cfsetospeed(&tty, baud_option);
    cfsetispeed(&tty, baud_option);

    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    tty.c_cflag |= (CS8 | CLOCAL | CREAD);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | IGNCR | INLCR);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        throw std::runtime_error(strerror(errno));
    }

    ret = tcflush(serial_fd, TCIOFLUSH);
    if (ret != 0) {
        throw std::runtime_error(strerror(errno));
    }
    return serial_fd;
}