//
// Created by user on 5/23/22.
//

#ifndef BUILD_SERIAL_HPP
#define BUILD_SERIAL_HPP
#include <termios.h>
namespace LS01 {
    int open_serial(const char*port, speed_t baud_option);
}
#endif //BUILD_SERIAL_HPP
